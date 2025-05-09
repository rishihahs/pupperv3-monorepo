#include <chrono>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class VelocityRecorder : public rclcpp::Node {
public:
  VelocityRecorder(double buffer_seconds = 10.0)
  : Node("velocity_recorder"), buffer_duration_(std::chrono::duration<double>(buffer_seconds)) {
    // Parameters
    this->declare_parameter("buffer_seconds", buffer_seconds);
    this->declare_parameter("recording_file", "velocity_recording.bin");
    
    buffer_duration_ = std::chrono::duration<double>(
      this->get_parameter("buffer_seconds").as_double());
    recording_file_ = this->get_parameter("recording_file").as_string();
    
    // Subscriber
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
      std::bind(&VelocityRecorder::cmd_vel_callback, this, std::placeholders::_1));
    
    // Services
    save_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_recording", 
      std::bind(&VelocityRecorder::save_recording, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    // Timer for buffer stats reporting
    stats_timer_ = this->create_wall_timer(
      5s, std::bind(&VelocityRecorder::report_stats, this));
                
    RCLCPP_INFO(this->get_logger(), "Velocity recorder initialized with %f second buffer", 
               buffer_seconds);
  }

private:
  struct TimestampedTwist {
    rclcpp::Time time;
    geometry_msgs::msg::Twist msg;
  };

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    auto now = this->now();
    
    // Lock while modifying buffer
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      cmd_vel_buffer_.push_back({now, *msg});
      
      // Track received message rate
      if (last_msg_time_.nanoseconds() > 0) {
        double dt = (now - last_msg_time_).seconds();
        if (dt > 0) {
          msg_rate_accumulator_ += 1.0 / dt;
          msg_rate_count_++;
        }
      }
      last_msg_time_ = now;
      
      // Remove old messages from buffer
      while (!cmd_vel_buffer_.empty()) {
        auto oldest = cmd_vel_buffer_.front().time;
        if ((now - oldest).seconds() > buffer_duration_.count()) {
          cmd_vel_buffer_.pop_front();
        } else {
          break;
        }
      }
    }
  }
  
  void report_stats() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    double avg_rate = 0.0;
    if (msg_rate_count_ > 0) {
      avg_rate = msg_rate_accumulator_ / msg_rate_count_;
      // Reset accumulators
      msg_rate_accumulator_ = 0.0;
      msg_rate_count_ = 0;
    }
    
    RCLCPP_INFO(this->get_logger(), 
               "Buffer stats - Size: %zu messages, Rate: %.2f Hz", 
               cmd_vel_buffer_.size(), avg_rate);
  }
  
  bool is_zero_velocity(const geometry_msgs::msg::Twist& twist_msg) {
    // Check if linear and angular velocities are close to zero
    const double epsilon = 0.001;
    return (std::abs(twist_msg.linear.x) < epsilon && 
            std::abs(twist_msg.linear.y) < epsilon &&
            std::abs(twist_msg.linear.z) < epsilon &&
            std::abs(twist_msg.angular.x) < epsilon &&
            std::abs(twist_msg.angular.y) < epsilon &&
            std::abs(twist_msg.angular.z) < epsilon);
  }
  
  void save_recording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::vector<TimestampedTwist> to_save;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      
      if (cmd_vel_buffer_.empty()) {
        response->success = false;
        response->message = "No velocity data in buffer";
        return;
      }
      
      // Find the last non-zero velocity command
      size_t last_non_zero = cmd_vel_buffer_.size() - 1;
      while (last_non_zero > 0) {
        if (!is_zero_velocity(cmd_vel_buffer_[last_non_zero].msg)) {
          break;
        }
        last_non_zero--;
      }
      
      // Prepare data for saving - make a copy to avoid holding the lock
      to_save.assign(cmd_vel_buffer_.begin(), cmd_vel_buffer_.begin() + last_non_zero + 1);
    }
    
    // Save to file
    try {
      std::ofstream outfile(recording_file_, std::ios::binary);
      
      // Write number of messages
      size_t num_msgs = to_save.size();
      outfile.write(reinterpret_cast<char*>(&num_msgs), sizeof(num_msgs));
      
      // Adjust time references - make first message time = 0 for consistent replay
      rclcpp::Time first_msg_time = to_save.empty() ? rclcpp::Time(0, 0) : to_save[0].time;
      
      // Write each message
      for (const auto& tj : to_save) {
        // Calculate time delta from first message
        rclcpp::Duration time_delta = tj.time - first_msg_time;
        int64_t nanosec_delta = time_delta.nanoseconds();
        outfile.write(reinterpret_cast<char*>(&nanosec_delta), sizeof(nanosec_delta));
        
        // Write linear velocity
        outfile.write(reinterpret_cast<const char*>(&tj.msg.linear.x), sizeof(double));
        outfile.write(reinterpret_cast<const char*>(&tj.msg.linear.y), sizeof(double));
        outfile.write(reinterpret_cast<const char*>(&tj.msg.linear.z), sizeof(double));
        
        // Write angular velocity
        outfile.write(reinterpret_cast<const char*>(&tj.msg.angular.x), sizeof(double));
        outfile.write(reinterpret_cast<const char*>(&tj.msg.angular.y), sizeof(double));
        outfile.write(reinterpret_cast<const char*>(&tj.msg.angular.z), sizeof(double));
      }
      
      outfile.close();
      response->success = true;
      response->message = "Saved " + std::to_string(to_save.size()) + 
                         " velocity messages to " + recording_file_;
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
      
      // Debug: print timing info for first few messages
      if (!to_save.empty()) {
        RCLCPP_INFO(this->get_logger(), "First message time: %.9f", 
                   to_save[0].time.seconds());
        
        if (to_save.size() > 1) {
          for (size_t i = 1; i < std::min(to_save.size(), size_t(5)); i++) {
            double dt = (to_save[i].time - to_save[i-1].time).seconds();
            RCLCPP_INFO(this->get_logger(), "Message %zu delta: %.9f seconds (%.1f Hz)", 
                       i, dt, dt > 0 ? 1.0/dt : 0.0);
          }
        }
      }
      
    } catch (const std::exception& e) {
      response->success = false;
      response->message = "Failed to save: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  std::deque<TimestampedTwist> cmd_vel_buffer_;
  std::chrono::duration<double> buffer_duration_;
  std::string recording_file_;
  std::mutex buffer_mutex_;
  
  // Stats tracking
  rclcpp::Time last_msg_time_{0, 0};
  double msg_rate_accumulator_{0.0};
  int msg_rate_count_{0};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelocityRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
