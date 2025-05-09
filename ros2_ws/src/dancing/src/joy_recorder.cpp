#include <chrono>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "joy_replay/srv/replay_joy.hpp"  // Custom service definition

using namespace std::chrono_literals;

class JoyRecorder : public rclcpp::Node {
public:
  JoyRecorder(double buffer_seconds = 10.0)
  : Node("joy_recorder"), buffer_duration_(std::chrono::duration<double>(buffer_seconds)) {
    // Parameters
    this->declare_parameter("buffer_seconds", buffer_seconds);
    this->declare_parameter("replay_file", "joy_replay_data.bin");
    
    buffer_duration_ = std::chrono::duration<double>(
      this->get_parameter("buffer_seconds").as_double());
    replay_file_ = this->get_parameter("replay_file").as_string();
    
    // Subscriber
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 100, 
      std::bind(&JoyRecorder::joy_callback, this, std::placeholders::_1));
    
    // Publisher for replaying
    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy_replay", 100);
    
    // Services
    save_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_joy_recording", 
      std::bind(&JoyRecorder::save_recording, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    replay_service_ = this->create_service<std_srvs::srv::Trigger>(
      "replay_joy", 
      std::bind(&JoyRecorder::replay_joy, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    RCLCPP_INFO(this->get_logger(), "Joy recorder initialized with %f second buffer", 
               buffer_seconds);
  }

private:
  struct TimestampedJoy {
    rclcpp::Time time;
    sensor_msgs::msg::Joy msg;
  };

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    auto now = this->now();
    joy_buffer_.push_back({now, *msg});
    
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
    
    // Prepare data for saving
    std::vector<TimestampedJoy> to_save(joy_buffer_.begin(), 
                                        joy_buffer_.begin() + last_non_zero + 1);
    
    // Save to file
    try {
      std::ofstream outfile(replay_file_, std::ios::binary);
      
      // Write number of messages
      size_t num_msgs = to_save.size();
      outfile.write(reinterpret_cast<char*>(&num_msgs), sizeof(num_msgs));
      
      // Write each message
      for (const auto& tj : to_save) {
        // Write timestamp
        int64_t sec = tj.time.seconds();
        uint32_t nanosec = tj.time.nanoseconds();
        outfile.write(reinterpret_cast<char*>(&sec), sizeof(sec));
        outfile.write(reinterpret_cast<char*>(&nanosec), sizeof(nanosec));
        
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
    } catch (const std::exception& e) {
      response->success = false;
      response->message = "Failed to save: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }
  
  void replay_joy(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
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
      
      // Read each message
      for (size_t i = 0; i < num_msgs; i++) {
        TimestampedJoy tj;
        
        // Read timestamp
        int64_t sec;
        uint32_t nanosec;
        infile.read(reinterpret_cast<char*>(&sec), sizeof(sec));
        infile.read(reinterpret_cast<char*>(&nanosec), sizeof(nanosec));
        tj.time = rclcpp::Time(sec, nanosec);
        
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
      
      // Start a thread for replay
      replay_thread_ = std::thread([this, replay_data]() {
        if (replay_data.empty()) {
          RCLCPP_WARN(this->get_logger(), "No joy messages to replay");
          return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Starting replay of %zu joy messages", 
                  replay_data.size());
        
        // Calculate time offsets from the first message
        auto start_time = this->now();
        auto first_msg_time = replay_data[0].time;
        
        for (size_t i = 0; i < replay_data.size(); i++) {
          // Calculate when this message should be published
          auto& tj = replay_data[i];
          auto time_offset = tj.time - first_msg_time;
          auto target_time = start_time + time_offset;
          
          // Sleep until it's time to publish this message
          auto now = this->now();
          if (target_time > now) {
            rclcpp::sleep_for(target_time - now);
          }
          
          // Publish the message
          joy_publisher_->publish(tj.msg);
          
          // Check if we should stop
          if (stop_replay_) {
            break;
          }
        }
        
        RCLCPP_INFO(this->get_logger(), "Replay completed");
        stop_replay_ = false;
      });
      
      replay_thread_.detach();  // Let it run independently
      
      response->success = true;
      response->message = "Started replaying " + std::to_string(num_msgs) + 
                         " joy messages";
    } catch (const std::exception& e) {
      response->success = false;
      response->message = "Failed to replay: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replay_service_;
  
  std::deque<TimestampedJoy> joy_buffer_;
  std::chrono::duration<double> buffer_duration_;
  std::string replay_file_;
  
  std::thread replay_thread_;
  std::atomic<bool> stop_replay_{false};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
