#include <chrono>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class JoyRecorder : public rclcpp::Node {
public:
  JoyRecorder(double buffer_seconds = 10.0)
  : Node("joy_recorder"), buffer_duration_(std::chrono::duration<double>(buffer_seconds)) {
    // Parameters
    this->declare_parameter("buffer_seconds", buffer_seconds);
    this->declare_parameter("replay_file", "joy_replay_data.bin");
    this->declare_parameter("replay_to_joy", false);  // Whether to replay back to /joy topic
    
    buffer_duration_ = std::chrono::duration<double>(
      this->get_parameter("buffer_seconds").as_double());
    replay_file_ = this->get_parameter("replay_file").as_string();
    replay_to_joy_ = this->get_parameter("replay_to_joy").as_bool();
    
    // Subscriber
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
      std::bind(&JoyRecorder::joy_callback, this, std::placeholders::_1));
    
    // Publishers for replaying
    joy_replay_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
    
    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
    
    // Services
    save_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_joy_recording", 
      std::bind(&JoyRecorder::save_recording, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    replay_service_ = this->create_service<std_srvs::srv::Trigger>(
      "replay_joy", 
      std::bind(&JoyRecorder::replay_joy, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    stop_replay_service_ = this->create_service<std_srvs::srv::Trigger>(
      "stop_joy_replay", 
      std::bind(&JoyRecorder::stop_replay, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    set_replay_target_service_ = this->create_service<std_srvs::srv::SetBool>(
      "set_replay_to_joy", 
      std::bind(&JoyRecorder::set_replay_target, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    // Timer for buffer stats reporting
    stats_timer_ = this->create_wall_timer(
      5s, std::bind(&JoyRecorder::report_stats, this));
                
    RCLCPP_INFO(this->get_logger(), "Joy recorder initialized with %f second buffer", 
               buffer_seconds);
  }

  ~JoyRecorder() {
    // Make sure to stop any active replay
    if (replay_active_) {
      stop_replay_ = true;
      if (replay_thread_.joinable()) {
        replay_thread_.join();
      }
    }
  }

private:
  struct TimestampedJoy {
    rclcpp::Time time;
    sensor_msgs::msg::Joy msg;
  };

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    auto now = this->now();
    
    // Lock while modifying buffer
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      joy_buffer_.push_back({now, *msg});
      
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
      while (!joy_buffer_.empty()) {
        auto oldest = joy_buffer_.front().time;
        if ((now - oldest).seconds() > buffer_duration_.count()) {
          joy_buffer_.pop_front();
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
               joy_buffer_.size(), avg_rate);
  }
  
  bool all_axes_zero(const sensor_msgs::msg::Joy& joy_msg) {
    for (const auto& axis : joy_msg.axes) {
      if (std::abs(axis) > 0.001) { // Small threshold to account for noise
        return false;
      }
    }
    return true;
  }
  
  void save_recording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::vector<TimestampedJoy> to_save;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      
      if (joy_buffer_.empty()) {
        response->success = false;
        response->message = "No joystick data in buffer";
        return;
      }
      
      // Find the last non-zero joystick command
      size_t last_non_zero = joy_buffer_.size() - 1;
      while (last_non_zero > 0) {
        if (!all_axes_zero(joy_buffer_[last_non_zero].msg)) {
          break;
        }
        last_non_zero--;
      }
      
      // Prepare data for saving - make a copy to avoid holding the lock
      to_save.assign(joy_buffer_.begin(), joy_buffer_.begin() + last_non_zero + 1);
    }
    
    // Save to file
    try {
      std::ofstream outfile(replay_file_, std::ios::binary);
      
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
        
        // Write axes
        size_t num_axes = tj.msg.axes.size();
        outfile.write(reinterpret_cast<char*>(&num_axes), sizeof(num_axes));
        if (num_axes > 0) {
          outfile.write(reinterpret_cast<const char*>(tj.msg.axes.data()), 
                       num_axes * sizeof(float));
        }
        
        // Write buttons
        size_t num_buttons = tj.msg.buttons.size();
        outfile.write(reinterpret_cast<char*>(&num_buttons), sizeof(num_buttons));
        if (num_buttons > 0) {
          outfile.write(reinterpret_cast<const char*>(tj.msg.buttons.data()), 
                       num_buttons * sizeof(int));
        }
      }
      
      outfile.close();
      response->success = true;
      response->message = "Saved " + std::to_string(to_save.size()) + 
                         " joy messages to " + replay_file_;
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
  
  void replay_joy(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (replay_active_) {
      response->success = false;
      response->message = "Replay already in progress. Stop current replay first.";
      return;
    }
    
    try {
      std::ifstream infile(replay_file_, std::ios::binary);
      if (!infile) {
        response->success = false;
        response->message = "Failed to open replay file: " + replay_file_;
        return;
      }
      
      // Read number of messages
      size_t num_msgs;
      infile.read(reinterpret_cast<char*>(&num_msgs), sizeof(num_msgs));
      
      std::vector<TimestampedJoy> replay_data;
      replay_data.reserve(num_msgs);
      
      // Baseline time for all messages
      rclcpp::Time base_time(0, 0);
      
      // Read each message
      for (size_t i = 0; i < num_msgs; i++) {
        TimestampedJoy tj;
        
        // Read time delta
        int64_t nanosec_delta;
        infile.read(reinterpret_cast<char*>(&nanosec_delta), sizeof(nanosec_delta));
        
        // Calculate timestamp relative to base_time
        tj.time = base_time + rclcpp::Duration(std::chrono::nanoseconds(nanosec_delta));
        
        // Read axes
        size_t num_axes;
        infile.read(reinterpret_cast<char*>(&num_axes), sizeof(num_axes));
        tj.msg.axes.resize(num_axes);
        if (num_axes > 0) {
          infile.read(reinterpret_cast<char*>(tj.msg.axes.data()), 
                     num_axes * sizeof(float));
        }
        
        // Read buttons
        size_t num_buttons;
        infile.read(reinterpret_cast<char*>(&num_buttons), sizeof(num_buttons));
        tj.msg.buttons.resize(num_buttons);
        if (num_buttons > 0) {
          infile.read(reinterpret_cast<char*>(tj.msg.buttons.data()), 
                     num_buttons * sizeof(int));
        }
        
        replay_data.push_back(tj);
      }
      
      infile.close();
      
      if (replay_data.empty()) {
        response->success = false;
        response->message = "No joy messages in replay file";
        return;
      }
      
      // Debug: print timing info for first few messages
      RCLCPP_INFO(this->get_logger(), "Preparing to replay %zu messages", replay_data.size());
      
      if (replay_data.size() > 1) {
        for (size_t i = 1; i < std::min(replay_data.size(), size_t(5)); i++) {
          double dt = (replay_data[i].time - replay_data[i-1].time).seconds();
          RCLCPP_INFO(this->get_logger(), "Replay message %zu delta: %.9f seconds (%.1f Hz)", 
                     i, dt, dt > 0 ? 1.0/dt : 0.0);
        }
      }
      
      // Start a thread for replay
      stop_replay_ = false;
      replay_active_ = true;
      replay_thread_ = std::thread([this, replay_data]() {
        RCLCPP_INFO(this->get_logger(), "Starting replay of %zu joy messages", 
                  replay_data.size());
        
        // Record the start time
        auto start_time = this->now();
        
        // Track timing accuracy
        double max_timing_error = 0.0;
        double sum_timing_error = 0.0;
        int msg_count = 0;
        
        for (size_t i = 0; i < replay_data.size(); i++) {
          // Exit if stopped
          if (stop_replay_) {
            RCLCPP_INFO(this->get_logger(), "Replay stopped after %zu/%zu messages", 
                       i, replay_data.size());
            break;
          }
          
          // Get target time for this message
          auto& tj = replay_data[i];
          auto target_time = start_time + rclcpp::Duration(std::chrono::nanoseconds(tj.time.nanoseconds()));
          
          // High-precision sleep until it's time to publish
          auto now = this->now();
          if (target_time > now) {
            while (this->now() < target_time) {
              // Busy-wait for extremely precise timing
              if (stop_replay_) break;
              
              // Short yield to avoid consuming 100% CPU
              std::this_thread::yield();
            }
          }
          
          // Calculate actual timing error
          auto actual_time = this->now();
          double timing_error = std::abs((actual_time - target_time).seconds());
          max_timing_error = std::max(max_timing_error, timing_error);
          sum_timing_error += timing_error;
          msg_count++;
          
          // Publish to appropriate topic
          if (replay_to_joy_) {
            joy_publisher_->publish(tj.msg);
          } else {
            joy_replay_publisher_->publish(tj.msg);
          }
          
          // Log timing periodically
          if (i % 100 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Replayed %zu/%zu messages, timing error: %.6f ms", 
                        i, replay_data.size(), timing_error * 1000.0);
          }
        }
        
        // Log timing stats
        double avg_timing_error = msg_count > 0 ? sum_timing_error / msg_count : 0.0;
        RCLCPP_INFO(this->get_logger(), "Replay completed. Timing stats: Avg error: %.3f ms, Max error: %.3f ms", 
                   avg_timing_error * 1000.0, max_timing_error * 1000.0);
        
        replay_active_ = false;
        stop_replay_ = false;
      });
      
      replay_thread_.detach();  // Let it run independently
      
      response->success = true;
      response->message = "Started replaying " + std::to_string(num_msgs) + 
                         " joy messages to " + (replay_to_joy_ ? "/joy" : "/joy_replay") + " topic";
      
    } catch (const std::exception& e) {
      response->success = false;
      response->message = "Failed to replay: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }
  
  void stop_replay(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (!replay_active_) {
      response->success = false;
      response->message = "No replay currently active";
      return;
    }
    
    stop_replay_ = true;
    
    response->success = true;
    response->message = "Stop signal sent to replay thread";
  }
  
  void set_replay_target(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    if (replay_active_) {
      response->success = false;
      response->message = "Cannot change replay target while replay is active";
      return;
    }
    
    replay_to_joy_ = request->data;
    
    response->success = true;
    response->message = "Replay target set to " + std::string(replay_to_joy_ ? "/joy" : "/joy_replay");
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;          // For /joy topic
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_replay_publisher_;   // For /joy_replay topic
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replay_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_replay_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_replay_target_service_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  std::deque<TimestampedJoy> joy_buffer_;
  std::chrono::duration<double> buffer_duration_;
  std::string replay_file_;
  std::mutex buffer_mutex_;
  
  std::thread replay_thread_;
  std::atomic<bool> stop_replay_{false};
  std::atomic<bool> replay_active_{false};
  std::atomic<bool> replay_to_joy_{false};
  
  // Stats tracking
  rclcpp::Time last_msg_time_{0, 0};
  double msg_rate_accumulator_{0.0};
  int msg_rate_count_{0};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
