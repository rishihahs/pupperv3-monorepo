#include <chrono>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <pthread.h>
#include <sys/mman.h>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class VelocityRecorder : public rclcpp::Node {
public:
  VelocityRecorder(double buffer_seconds = 10.0)
  : Node("velocity_recorder"), buffer_duration_(std::chrono::duration<double>(buffer_seconds)) {
    // Parameters
    this->declare_parameter("buffer_seconds", buffer_seconds);
    this->declare_parameter("replay_file", "velocity_replay_data.bin");
    this->declare_parameter("replay_to_cmd_vel", false);
    this->declare_parameter("replay_time_factor", 1.0);
    
    buffer_duration_ = std::chrono::duration<double>(
      this->get_parameter("buffer_seconds").as_double());
    replay_file_ = this->get_parameter("replay_file").as_string();
    replay_to_cmd_vel_ = this->get_parameter("replay_to_cmd_vel").as_bool();
    replay_time_factor_ = this->get_parameter("replay_time_factor").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Replay time factor set to %.3f", replay_time_factor_);
    
    // Subscriber
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
      std::bind(&VelocityRecorder::cmd_vel_callback, this, std::placeholders::_1));
    
    // Publishers
    cmd_vel_replay_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_replay", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
    
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
    
    // Services
    save_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_velocity_recording", 
      std::bind(&VelocityRecorder::save_recording, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    replay_service_ = this->create_service<std_srvs::srv::Trigger>(
      "replay_velocity", 
      std::bind(&VelocityRecorder::replay_velocity, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    stop_replay_service_ = this->create_service<std_srvs::srv::Trigger>(
      "stop_velocity_replay", 
      std::bind(&VelocityRecorder::stop_replay, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    set_replay_target_service_ = this->create_service<std_srvs::srv::SetBool>(
      "set_replay_to_cmd_vel", 
      std::bind(&VelocityRecorder::set_replay_target, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    stats_service_ = this->create_service<std_srvs::srv::Trigger>(
      "velocity_recorder_stats", 
      std::bind(&VelocityRecorder::report_stats_service, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    // Timer for buffer stats reporting
    stats_timer_ = this->create_wall_timer(
      5s, std::bind(&VelocityRecorder::report_stats, this));
                
    RCLCPP_INFO(this->get_logger(), "Velocity recorder initialized with %f second buffer", 
               buffer_seconds);
  }

  ~VelocityRecorder() {
    // Make sure to stop any active replay
    if (replay_active_) {
      stop_replay_ = true;
      if (replay_thread_.joinable()) {
        replay_thread_.join();
      }
    }
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
    
    current_input_hz_ = avg_rate;
    
    RCLCPP_INFO(this->get_logger(), 
               "Buffer stats - Size: %zu messages, Rate: %.2f Hz", 
               cmd_vel_buffer_.size(), avg_rate);
  }
  
  void report_stats_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::stringstream ss;
    ss << "Velocity Recorder Performance Metrics:\n";
    
    // Message frequency stats
    ss << "Current input frequency: " << current_input_hz_ << " Hz\n";
    ss << "Replay frequency: " << replay_hz_ << " Hz\n";
    
    // Timing precision
    ss << "Replay timing error: avg=" << avg_timing_error_ms_ << "ms, max=" 
       << max_timing_error_ms_ << "ms\n";
    
    // Buffer stats
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      ss << "Buffer utilization: " << cmd_vel_buffer_.size() << " messages\n";
    }
    
    // Replay settings
    ss << "Replay destination: " << (replay_to_cmd_vel_ ? "/cmd_vel" : "/cmd_vel_replay") << "\n";
    ss << "Replay time factor: " << replay_time_factor_ << "\n";
    
    response->success = true;
    response->message = ss.str();
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
                         " velocity messages to " + replay_file_;
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
  
  void replay_velocity(
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
      
      std::vector<TimestampedTwist> replay_data;
      replay_data.reserve(num_msgs);
      
      // Baseline time for all messages
      rclcpp::Time base_time(0, 0);
      
      // Read each message
      for (size_t i = 0; i < num_msgs; i++) {
        TimestampedTwist tj;
        
        // Read time delta
        int64_t nanosec_delta;
        infile.read(reinterpret_cast<char*>(&nanosec_delta), sizeof(nanosec_delta));
        
        // Calculate timestamp relative to base_time
        tj.time = base_time + rclcpp::Duration(std::chrono::nanoseconds(nanosec_delta));
        
        // Read linear velocity
        infile.read(reinterpret_cast<char*>(&tj.msg.linear.x), sizeof(double));
        infile.read(reinterpret_cast<char*>(&tj.msg.linear.y), sizeof(double));
        infile.read(reinterpret_cast<char*>(&tj.msg.linear.z), sizeof(double));
        
        // Read angular velocity
        infile.read(reinterpret_cast<char*>(&tj.msg.angular.x), sizeof(double));
        infile.read(reinterpret_cast<char*>(&tj.msg.angular.y), sizeof(double));
        infile.read(reinterpret_cast<char*>(&tj.msg.angular.z), sizeof(double));
        
        replay_data.push_back(tj);
      }
      
      infile.close();
      
      if (replay_data.empty()) {
        response->success = false;
        response->message = "No velocity messages in replay file";
        return;
      }
      
      // Debug: print timing info for first few messages
      RCLCPP_INFO(this->get_logger(), "Preparing to replay %zu messages", replay_data.size());
      RCLCPP_INFO(this->get_logger(), "Using time factor: %.3f", replay_time_factor_);
      
      if (replay_data.size() > 1) {
        for (size_t i = 1; i < std::min(replay_data.size(), size_t(5)); i++) {
          double dt = (replay_data[i].time - replay_data[i-1].time).seconds();
          double adjusted_dt = dt * replay_time_factor_;
          RCLCPP_INFO(this->get_logger(), 
                     "Replay message %zu delta: %.6f seconds (%.1f Hz) -> Adjusted: %.6f seconds (%.1f Hz)", 
                     i, dt, dt > 0 ? 1.0/dt : 0.0, 
                     adjusted_dt, adjusted_dt > 0 ? 1.0/adjusted_dt : 0.0);
        }
      }
      
      // Reset timing stats
      max_timing_error_ms_ = 0.0;
      avg_timing_error_ms_ = 0.0;
      replay_hz_ = 0.0;
      
      // Start a thread for replay
      stop_replay_ = false;
      replay_active_ = true;
      replay_thread_ = std::thread([this, replay_data]() {
        // Set this thread to real-time priority
        sched_param sch_params;
        sch_params.sched_priority = sched_get_priority_max(SCHED_FIFO);
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch_params) != 0) {
          RCLCPP_WARN(this->get_logger(), 
                     "Failed to set thread to real-time priority. Replay timing may be less accurate.");
        }
        
        RCLCPP_INFO(this->get_logger(), "Starting replay of %zu velocity messages", 
                  replay_data.size());
        
        // Record the start time
        auto start_time = this->now();
        auto last_publish_time = start_time;
        
        // Track timing accuracy
        double max_timing_error = 0.0;
        double sum_timing_error = 0.0;
        int msg_count = 0;
        
        // For rate calculation
        int msgs_in_window = 0;
        auto window_start_time = start_time;
        
        for (size_t i = 0; i < replay_data.size(); i++) {
          // Exit if stopped
          if (stop_replay_) {
            RCLCPP_INFO(this->get_logger(), "Replay stopped after %zu/%zu messages", 
                       i, replay_data.size());
            break;
          }
          
          // Get target time for this message
          const auto& tj = replay_data[i];
          
          // Apply time factor for speed adjustment
          double scaled_delta = tj.time.seconds() * replay_time_factor_;
          auto target_time = start_time + rclcpp::Duration::from_seconds(scaled_delta);
          
          // Get current time
          auto now = this->now();
          
          // High-precision sleep until it's time to publish
          if (target_time > now) {
            // Calculate sleep duration
            auto time_until = target_time - now;
            
            // Use high-precision sleep
            struct timespec req, rem;
            req.tv_sec = time_until.seconds();
            req.tv_nsec = time_until.nanoseconds() % 1000000000;
            
            // Use clock_nanosleep for highest precision
            clock_nanosleep(CLOCK_MONOTONIC, 0, &req, &rem);
          }
          
          // Create a copy of the message that we can modify if needed
          geometry_msgs::msg::Twist message_copy = tj.msg;
          
          // Publish to appropriate topic
          if (replay_to_cmd_vel_) {
            cmd_vel_publisher_->publish(message_copy);
          } else {
            cmd_vel_replay_publisher_->publish(message_copy);
          }
          
          // Calculate actual timing error
          auto actual_time = this->now();
          double timing_error = std::abs((actual_time - target_time).seconds());
          max_timing_error = std::max(max_timing_error, timing_error);
          sum_timing_error += timing_error;
          msg_count++;
          
          // Update rate calculation
          msgs_in_window++;
          double window_duration = (actual_time - window_start_time).seconds();
          if (window_duration >= 1.0) {
            // Update replay rate
            replay_hz_ = msgs_in_window / window_duration;
            msgs_in_window = 0;
            window_start_time = actual_time;
          }
          
          // Log timing periodically
          if (i % 100 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Replayed %zu/%zu messages, timing error: %.6f ms", 
                        i, replay_data.size(), timing_error * 1000.0);
          }
          
          // Update last publish time
          last_publish_time = actual_time;
        }
        
        // Log timing stats
        double avg_timing_error = msg_count > 0 ? sum_timing_error / msg_count : 0.0;
        max_timing_error_ms_ = max_timing_error * 1000.0;
        avg_timing_error_ms_ = avg_timing_error * 1000.0;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Replay completed. Timing stats: Avg error: %.3f ms, Max error: %.3f ms", 
                   avg_timing_error_ms_, max_timing_error_ms_);
        
        replay_active_ = false;
        stop_replay_ = false;
      });
      
      replay_thread_.detach();  // Let it run independently
      
      response->success = true;
      response->message = "Started replaying " + std::to_string(num_msgs) + 
                         " velocity messages to " + (replay_to_cmd_vel_ ? "/cmd_vel" : "/cmd_vel_replay") + " topic";
      
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
    
    replay_to_cmd_vel_ = request->data;
    
    response->success = true;
    response->message = "Replay target set to " + std::string(replay_to_cmd_vel_ ? "/cmd_vel" : "/cmd_vel_replay");
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;       // For /cmd_vel topic
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_replay_publisher_; // For /cmd_vel_replay topic
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replay_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_replay_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_replay_target_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stats_service_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  std::deque<TimestampedTwist> cmd_vel_buffer_;
  std::chrono::duration<double> buffer_duration_;
  std::string replay_file_;
  std::mutex buffer_mutex_;
  
  std::thread replay_thread_;
  std::atomic<bool> stop_replay_{false};
  std::atomic<bool> replay_active_{false};
  std::atomic<bool> replay_to_cmd_vel_{false};
  
  // Configurable time factor for replay
  double replay_time_factor_{1.0};
  
  // Stats tracking
  rclcpp::Time last_msg_time_{0, 0};
  double msg_rate_accumulator_{0.0};
  int msg_rate_count_{0};
  
  // Performance metrics
  double current_input_hz_{0.0};
  double replay_hz_{0.0};
  double avg_timing_error_ms_{0.0};
  double max_timing_error_ms_{0.0};
};

int main(int argc, char * argv[]) {
  // Lock memory to prevent page faults during real-time operation
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    std::cerr << "Warning: Failed to lock memory. Replay timing may be less accurate." << std::endl;
  }
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelocityRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
