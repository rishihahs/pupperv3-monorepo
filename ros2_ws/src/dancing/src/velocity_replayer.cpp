#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <pthread.h>
#include <sys/mman.h>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

struct TimestampedTwist {
  rclcpp::Time time;  // Time relative to sequence start
  geometry_msgs::msg::Twist msg;
};

class VelocityReplayer : public rclcpp::Node {
public:
  VelocityReplayer() : Node("velocity_replayer") {
    // Parameters
    this->declare_parameter("replay_file", "velocity_recording.bin");
    this->declare_parameter("replay_to_cmd_vel", true);
    this->declare_parameter("replay_time_factor", 1.0);
    this->declare_parameter("loop_replay", true);
    this->declare_parameter("loop_pause_time", 0.1);  // Time in seconds to pause between loops
    this->declare_parameter("trim_start_seconds", 1.0);  // Time in seconds to trim from start
    this->declare_parameter("trim_end_seconds", 0.0);    // Time in seconds to trim from end

    trim_end_seconds_ = this->get_parameter("trim_end_seconds").as_double();
    trim_start_seconds_ = this->get_parameter("trim_start_seconds").as_double();
    replay_file_ = this->get_parameter("replay_file").as_string();
    replay_to_cmd_vel_ = this->get_parameter("replay_to_cmd_vel").as_bool();
    replay_time_factor_ = this->get_parameter("replay_time_factor").as_double();
    loop_replay_ = this->get_parameter("loop_replay").as_bool();
    loop_pause_time_ = this->get_parameter("loop_pause_time").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Settings: time_factor=%.3f, loop=%s, pause=%.2fs",
              replay_time_factor_, loop_replay_ ? "true" : "false", loop_pause_time_);
    
    // Publishers
    cmd_vel_replay_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_replay", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
    
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
    
    // Services
    start_replay_service_ = this->create_service<std_srvs::srv::Trigger>(
      "start_replay", 
      std::bind(&VelocityReplayer::start_replay, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    stop_replay_service_ = this->create_service<std_srvs::srv::Trigger>(
      "stop_replay", 
      std::bind(&VelocityReplayer::stop_replay, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    set_replay_target_service_ = this->create_service<std_srvs::srv::SetBool>(
      "set_replay_to_cmd_vel", 
      std::bind(&VelocityReplayer::set_replay_target, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    set_loop_service_ = this->create_service<std_srvs::srv::SetBool>(
      "set_loop_replay", 
      std::bind(&VelocityReplayer::set_loop_replay, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    stats_service_ = this->create_service<std_srvs::srv::Trigger>(
      "replay_stats", 
      std::bind(&VelocityReplayer::report_stats, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    RCLCPP_INFO(this->get_logger(), "Velocity replayer initialized");
    
    // Try to load the replay file on startup
    load_replay_file();
  }

  ~VelocityReplayer() {
    // Make sure to stop any active replay
    if (replay_active_) {
      stop_replay_ = true;
      if (replay_thread_.joinable()) {
        replay_thread_.join();
      }
    }
  }

private:
  bool load_replay_file() {
    try {
        std::ifstream infile(replay_file_, std::ios::binary);
        if (!infile) {
            RCLCPP_WARN(this->get_logger(), "Failed to open replay file: %s", replay_file_.c_str());
            return false;
        }
        
        // Read number of messages
        size_t num_msgs;
        infile.read(reinterpret_cast<char*>(&num_msgs), sizeof(num_msgs));
        
        // Temporary vector to hold all loaded data
        std::vector<TimestampedTwist> temp_data;
        temp_data.reserve(num_msgs);
        
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
            
            temp_data.push_back(tj);
        }
        
        infile.close();
        
        if (temp_data.empty()) {
            RCLCPP_WARN(this->get_logger(), "No velocity messages in replay file");
            return false;
        }

        // Get total sequence duration before trimming
        double total_duration = 0.0;
        if (temp_data.size() > 1) {
            total_duration = temp_data.back().time.seconds() - temp_data.front().time.seconds();
        }

        // Now apply the trimming
        replay_data_.clear();
        
        // Calculate trim thresholds
        double start_trim_threshold = trim_start_seconds_;
        double end_trim_threshold = total_duration - trim_end_seconds_;
        
        // Make sure end_trim_threshold is greater than start_trim_threshold
        if (end_trim_threshold <= start_trim_threshold) {
            RCLCPP_WARN(this->get_logger(), 
                      "Trim parameters would remove entire sequence! Using only start trim.");
            end_trim_threshold = total_duration;
        }
        
        // Find valid message indices after applying trim
        size_t first_included_idx = 0;
        size_t last_included_idx = temp_data.size() - 1;
        
        // Find first message after start trim point
        if (start_trim_threshold > 0.0) {
            for (size_t i = 0; i < temp_data.size(); i++) {
                if (temp_data[i].time.seconds() >= start_trim_threshold) {
                    first_included_idx = i;
                    break;
                }
            }
        }
        
        // Find last message before end trim point
        if (trim_end_seconds_ > 0.0) {
            for (size_t i = temp_data.size(); i-- > 0; ) {
                if (temp_data[i].time.seconds() <= end_trim_threshold) {
                    last_included_idx = i;
                    break;
                }
            }
        }
        
        // Make sure we have valid indices
        if (first_included_idx > last_included_idx) {
            RCLCPP_ERROR(this->get_logger(), "Invalid trim indices: %zu to %zu", 
                       first_included_idx, last_included_idx);
            return false;
        }
        
        // If we're trimming the start, rebase the times
        rclcpp::Time new_base_time = (first_included_idx > 0) ? 
                                    temp_data[first_included_idx].time : 
                                    rclcpp::Time(0, 0);
        
        // Add all messages within the valid range with adjusted timestamps
        for (size_t i = first_included_idx; i <= last_included_idx; i++) {
            TimestampedTwist tj = temp_data[i];
            
            // Adjust timestamp to be relative to the new base time
            tj.time = rclcpp::Time(0, 0) + (tj.time - new_base_time);
            
            replay_data_.push_back(tj);
        }
        
        // Debug: print timing info
        size_t start_trimmed = first_included_idx;
        size_t end_trimmed = temp_data.size() - last_included_idx - 1;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Loaded %zu messages from %s (trimmed %zu from start, %zu from end)", 
                   replay_data_.size(), replay_file_.c_str(), 
                   start_trimmed, end_trimmed);
        
        // Calculate total duration of the trimmed replay sequence
        if (replay_data_.size() > 1) {
            double duration = replay_data_.back().time.seconds() - replay_data_.front().time.seconds();
            RCLCPP_INFO(this->get_logger(), 
                      "Original duration: %.2f seconds, Trimmed duration: %.2f seconds", 
                      total_duration, duration);
        }
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load replay file: %s", e.what());
        return false;
    }
  }
  
  void report_stats(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::stringstream ss;
    ss << "Velocity Replayer Status:\n";
    
    // Replay file info
    ss << "Replay file: " << replay_file_ << "\n";
    ss << "Number of messages: " << replay_data_.size() << "\n";
    
    if (replay_data_.size() > 1) {
      double duration = replay_data_.back().time.seconds() - replay_data_.front().time.seconds();
      ss << "Sequence duration: " << duration << " seconds\n";
    }
    
    // Current state
    ss << "Replay active: " << (replay_active_ ? "Yes" : "No") << "\n";
    ss << "Looping enabled: " << (loop_replay_ ? "Yes" : "No") << "\n";
    ss << "Loop pause time: " << loop_pause_time_ << " seconds\n";
    ss << "Time factor: " << replay_time_factor_ << "x\n";
    ss << "Target topic: " << (replay_to_cmd_vel_ ? "/cmd_vel" : "/cmd_vel_replay") << "\n";
    
    // Performance metrics if available
    ss << "Replay frequency: " << replay_hz_ << " Hz\n";
    ss << "Timing precision - Avg error: " << avg_timing_error_ms_ << " ms, ";
    ss << "Max error: " << max_timing_error_ms_ << " ms\n";
    ss << "Loop count: " << loop_count_ << "\n";
    
    response->success = true;
    response->message = ss.str();
  }
  
  void start_replay(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (replay_active_) {
      response->success = false;
      response->message = "Replay already in progress. Stop current replay first.";
      return;
    }
    
    // Try to load the file if we don't have data already
    if (replay_data_.empty() && !load_replay_file()) {
      response->success = false;
      response->message = "Failed to load replay data.";
      return;
    }
    
    // Reset timing stats and loop count
    max_timing_error_ms_ = 0.0;
    avg_timing_error_ms_ = 0.0;
    replay_hz_ = 0.0;
    loop_count_ = 0;
    
    // Start the replay thread
    stop_replay_ = false;
    replay_active_ = true;
    replay_thread_ = std::thread([this]() {
      // Set this thread to real-time priority
      sched_param sch_params;
      sch_params.sched_priority = sched_get_priority_max(SCHED_FIFO);
      if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch_params) != 0) {
        RCLCPP_WARN(this->get_logger(), 
                   "Failed to set thread to real-time priority. Replay timing may be less accurate.");
      }
      
      RCLCPP_INFO(this->get_logger(), "Starting replay of %zu velocity messages", 
                replay_data_.size());
      
      // Loop control
      bool continue_replay = true;
      
      while (continue_replay && !stop_replay_) {
        // Perform one full replay sequence
        if (!replay_sequence()) {
          // If replay_sequence returns false, it was interrupted
          break;
        }
        
        // Increase loop count
        loop_count_++;
        
        // Should we continue looping?
        if (!loop_replay_) {
          break;
        }
        
        // Publish zero velocity command at the end of each loop
        if (loop_pause_time_ > 0.0) {
          publish_zero_velocity();
        }
        
        // Pause between loops
        if (loop_pause_time_ > 0.0) {
          RCLCPP_INFO(this->get_logger(), "Loop %d completed. Pausing for %.2f seconds before next loop.",
                    loop_count_, loop_pause_time_);
          
          // Use high-precision sleep
          double remaining_pause = loop_pause_time_;
          while (remaining_pause > 0.0 && !stop_replay_) {
            double sleep_chunk = std::min(remaining_pause, 0.1);  // Sleep in small chunks to check stop flag
            struct timespec req;
            req.tv_sec = static_cast<time_t>(sleep_chunk);
            req.tv_nsec = static_cast<long>((sleep_chunk - req.tv_sec) * 1e9);
            nanosleep(&req, nullptr);
            remaining_pause -= sleep_chunk;
          }
        }
      }
      
      // Ensure we publish a zero velocity when stopping
      publish_zero_velocity();
      
      RCLCPP_INFO(this->get_logger(), "Replay finished after %d loops", loop_count_);
      
      replay_active_ = false;
      stop_replay_ = false;
    });
    
    replay_thread_.detach();  // Let it run independently
    
    response->success = true;

    std::stringstream ss;
    ss << "Started replaying velocity commands ";
    ss << (loop_replay_ ? "with looping enabled" : "once");
    ss << " to ";
    ss << (replay_to_cmd_vel_ ? "/cmd_vel" : "/cmd_vel_replay");
    response->message = ss.str();
  }
  
  bool replay_sequence() {
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
    
    for (size_t i = 0; i < replay_data_.size(); i++) {
      // Exit if stopped
      if (stop_replay_) {
        RCLCPP_INFO(this->get_logger(), "Replay stopped after %zu/%zu messages", 
                   i, replay_data_.size());
        return false;
      }
      
      // Get target time for this message
      const auto& tj = replay_data_[i];
      
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
                    i, replay_data_.size(), timing_error * 1000.0);
      }
      
      // Update last publish time
      last_publish_time = actual_time;
    }
    
    // Update timing stats
    if (msg_count > 0) {
      double avg_timing_error = sum_timing_error / msg_count;
      max_timing_error_ms_ = max_timing_error * 1000.0;
      avg_timing_error_ms_ = avg_timing_error * 1000.0;
      
      RCLCPP_INFO(this->get_logger(), "Sequence completed. Timing: Avg error=%.3f ms, Max error=%.3f ms", 
                 avg_timing_error_ms_, max_timing_error_ms_);
    }
    
    return true;  // Completed successfully
  }
  
  void publish_zero_velocity() {
    geometry_msgs::msg::Twist zero_msg;
    zero_msg.linear.x = 0.0;
    zero_msg.linear.y = 0.0;
    zero_msg.linear.z = 0.0;
    zero_msg.angular.x = 0.0;
    zero_msg.angular.y = 0.0;
    zero_msg.angular.z = 0.0;
    
    // Publish to the same topic we've been using
    if (replay_to_cmd_vel_) {
      cmd_vel_publisher_->publish(zero_msg);
    } else {
      cmd_vel_replay_publisher_->publish(zero_msg);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Published zero velocity command");
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
  
  void set_loop_replay(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    loop_replay_ = request->data;
    
    response->success = true;
    response->message = std::string(loop_replay_ ? "Enabled" : "Disabled") + " loop replay";
    
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;       // For /cmd_vel topic
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_replay_publisher_; // For /cmd_vel_replay topic
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_replay_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_replay_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_replay_target_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_loop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stats_service_;
  
  std::string replay_file_;
  std::vector<TimestampedTwist> replay_data_;
  
  double trim_start_seconds_{0.0};
  double trim_end_seconds_{0.0};
  std::thread replay_thread_;
  std::atomic<bool> stop_replay_{false};
  std::atomic<bool> replay_active_{false};
  std::atomic<bool> replay_to_cmd_vel_{false};
  std::atomic<bool> loop_replay_{false};
  double loop_pause_time_{1.0};
  
  // Configurable time factor for replay
  double replay_time_factor_{1.0};
  
  // Performance metrics
  double replay_hz_{0.0};
  double avg_timing_error_ms_{0.0};
  double max_timing_error_ms_{0.0};
  int loop_count_{0};
};

int main(int argc, char * argv[]) {
  // Lock memory to prevent page faults during real-time operation
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    std::cerr << "Warning: Failed to lock memory. Replay timing may be less accurate." << std::endl;
  }
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelocityReplayer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
