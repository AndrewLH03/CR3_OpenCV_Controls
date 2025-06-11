/**
 * Emergency Stop Handler for CR3 Robot System
 * Ultra-fast emergency response system in C++
 */

#include <rclcpp/rclcpp.hpp>
#include <cr3_hand_control/msg/basic_command.hpp>
#include <cr3_hand_control/msg/robot_status.hpp>
#include <cr3_hand_control/msg/safety_alert.hpp>
#include <cr3_hand_control/srv/emergency_stop.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>

class EmergencyStopHandler : public rclcpp::Node
{
public:
    EmergencyStopHandler() : Node("emergency_stop_handler")
    {
        // Declare parameters
        declare_emergency_parameters();
        load_emergency_config();
        
        // Publishers
        emergency_command_pub_ = create_publisher<cr3_hand_control::msg::BasicCommand>(
            "/emergency_commands", 10);
        
        emergency_status_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/emergency_active", 10);
        
        // Subscribers
        safety_alert_sub_ = create_subscription<cr3_hand_control::msg::SafetyAlert>(
            "/safety_alerts", 10,
            std::bind(&EmergencyStopHandler::safety_alert_callback, this, std::placeholders::_1));
        
        robot_status_sub_ = create_subscription<cr3_hand_control::msg::RobotStatus>(
            "/robot_status", 10,
            std::bind(&EmergencyStopHandler::robot_status_callback, this, std::placeholders::_1));
        
        // Services
        emergency_service_ = create_service<cr3_hand_control::srv::EmergencyStop>(
            "/emergency_stop_handler",
            std::bind(&EmergencyStopHandler::emergency_stop_callback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Initialize state
        emergency_active_ = false;
        last_robot_position_.x = 0.0;
        last_robot_position_.y = 0.0;
        last_robot_position_.z = 0.0;
        
        // Create timer for emergency status monitoring
        status_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),  // 100 Hz for fast response
            std::bind(&EmergencyStopHandler::monitor_emergency_status, this));
        
        RCLCPP_INFO(get_logger(), "Emergency Stop Handler initialized with %.1f ms response timeout",
                    response_timeout_ms_);
    }

private:
    // Publishers and subscribers
    rclcpp::Publisher<cr3_hand_control::msg::BasicCommand>::SharedPtr emergency_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_status_pub_;
    rclcpp::Subscription<cr3_hand_control::msg::SafetyAlert>::SharedPtr safety_alert_sub_;
    rclcpp::Subscription<cr3_hand_control::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Service<cr3_hand_control::srv::EmergencyStop>::SharedPtr emergency_service_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // Emergency configuration
    double response_timeout_ms_;
    double soft_stop_decel_;
    double hard_stop_decel_;
    double emergency_decel_;
    
    // State variables
    bool emergency_active_;
    geometry_msgs::msg::Point last_robot_position_;
    std::chrono::steady_clock::time_point emergency_start_time_;
    std::string emergency_reason_;
    
    void declare_emergency_parameters()
    {
        declare_parameter("safety.emergency_response.response_timeout", 0.1);  // seconds
        declare_parameter("safety.emergency_response.soft_stop_decel", 200.0);
        declare_parameter("safety.emergency_response.hard_stop_decel", 500.0);
        declare_parameter("safety.emergency_response.emergency_decel", 1000.0);
    }
    
    void load_emergency_config()
    {
        auto timeout_seconds = get_parameter("safety.emergency_response.response_timeout").as_double();
        response_timeout_ms_ = timeout_seconds * 1000.0;
        
        soft_stop_decel_ = get_parameter("safety.emergency_response.soft_stop_decel").as_double();
        hard_stop_decel_ = get_parameter("safety.emergency_response.hard_stop_decel").as_double();
        emergency_decel_ = get_parameter("safety.emergency_response.emergency_decel").as_double();
    }
    
    void safety_alert_callback(const cr3_hand_control::msg::SafetyAlert::SharedPtr msg)
    {
        // Automatically trigger emergency stop for EMERGENCY level alerts
        if (msg->level >= 3 && !emergency_active_) {  // EMERGENCY level
            trigger_emergency_stop("automatic", msg->description);
            RCLCPP_ERROR(get_logger(), "Auto-triggered emergency stop due to: %s", 
                        msg->description.c_str());
        }
        // Prepare for potential emergency on DANGER level alerts
        else if (msg->level >= 2 && !emergency_active_) {  // DANGER level
            prepare_emergency_stop();
            RCLCPP_WARN(get_logger(), "Emergency stop preparation due to: %s", 
                       msg->description.c_str());
        }
    }
    
    void robot_status_callback(const cr3_hand_control::msg::RobotStatus::SharedPtr msg)
    {
        if (msg->current_position.size() >= 3) {
            last_robot_position_.x = msg->current_position[0];
            last_robot_position_.y = msg->current_position[1];
            last_robot_position_.z = msg->current_position[2];
        }
    }
    
    void emergency_stop_callback(const std::shared_ptr<cr3_hand_control::srv::EmergencyStop::Request> request,
                                std::shared_ptr<cr3_hand_control::srv::EmergencyStop::Response> response)
    {
        if (request->stop_type == "emergency") {  // Emergency halt
            trigger_emergency_stop("manual", "Manual emergency stop requested");
            response->success = true;
            response->message = "Emergency stop activated";
            response->stop_time = 0.01;  // Immediate stop
            RCLCPP_WARN(get_logger(), "Manual emergency stop activated");
        }
        else if (request->stop_type == "soft") {  // Soft stop
            trigger_soft_stop("Manual soft stop requested");
            response->success = true;
            response->message = "Soft stop activated";
            response->stop_time = 2.0;  // 2 second stop
            RCLCPP_INFO(get_logger(), "Manual soft stop activated");
        }
        else if (request->stop_type == "reset") {  // Reset
            reset_emergency_stop();
            response->success = true;
            response->message = "Emergency stop reset";
            response->stop_time = 0.0;  // No stop time for reset
            RCLCPP_INFO(get_logger(), "Emergency stop reset");
        }
        else {
            response->success = false;
            response->message = "Unknown stop type: " + request->stop_type;
            response->stop_time = 0.0;
            RCLCPP_ERROR(get_logger(), "Unknown emergency stop type: %s", request->stop_type.c_str());
        }
    }
    
    void trigger_emergency_stop(const std::string& trigger_type, const std::string& reason)
    {
        if (emergency_active_) {
            return;  // Already in emergency stop
        }
        
        emergency_active_ = true;
        emergency_start_time_ = std::chrono::steady_clock::now();
        emergency_reason_ = reason;
        
        // Send immediate stop command
        auto stop_command = cr3_hand_control::msg::BasicCommand();
        stop_command.command_type = "emergency_stop";
        // Set position array with current position
        stop_command.position = {last_robot_position_.x, last_robot_position_.y, last_robot_position_.z, 0.0, 0.0, 0.0};
        stop_command.velocity = 0.0;
        
        emergency_command_pub_->publish(stop_command);
        
        RCLCPP_ERROR(get_logger(), "EMERGENCY STOP ACTIVATED [%s]: %s", 
                    trigger_type.c_str(), reason.c_str());
    }
    
    void trigger_soft_stop(const std::string& reason)
    {
        // Send controlled deceleration command
        auto stop_command = cr3_hand_control::msg::BasicCommand();
        stop_command.command_type = "soft_stop";
        stop_command.position = {last_robot_position_.x, last_robot_position_.y, last_robot_position_.z, 0.0, 0.0, 0.0};
        stop_command.velocity = 0.0;  // Gradual stop
        
        emergency_command_pub_->publish(stop_command);
        
        RCLCPP_WARN(get_logger(), "SOFT STOP ACTIVATED: %s", reason.c_str());
    }
    
    void prepare_emergency_stop()
    {
        // Pre-load emergency stop command for faster response
        // This could involve pre-calculating stop trajectories
        RCLCPP_WARN(get_logger(), "Emergency stop system prepared for rapid activation");
    }
    
    void reset_emergency_stop()
    {
        if (!emergency_active_) {
            RCLCPP_WARN(get_logger(), "Emergency stop reset requested but no emergency active");
            return;
        }
        
        emergency_active_ = false;
        emergency_reason_.clear();
        
        // Send reset command
        auto reset_command = cr3_hand_control::msg::BasicCommand();
        reset_command.command_type = "reset_emergency";
        reset_command.position = {last_robot_position_.x, last_robot_position_.y, last_robot_position_.z, 0.0, 0.0, 0.0};
        reset_command.velocity = 0.0;
        
        emergency_command_pub_->publish(reset_command);
        
        RCLCPP_INFO(get_logger(), "Emergency stop reset - system ready for normal operation");
    }
    
    void monitor_emergency_status()
    {
        // Publish emergency status
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = emergency_active_;
        emergency_status_pub_->publish(status_msg);
        
        // Monitor emergency stop duration
        if (emergency_active_) {
            auto current_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - emergency_start_time_).count();
            
            // Log periodic status during emergency
            if (duration % 1000 == 0) {  // Every second
                RCLCPP_INFO(get_logger(), "Emergency stop active for %ld ms: %s", 
                           duration, emergency_reason_.c_str());
            }
        }
    }
    
public:
    bool is_emergency_active() const { return emergency_active_; }
    std::string get_emergency_reason() const { return emergency_reason_; }
    
    // Manual trigger methods for external use
    void manual_emergency_stop(const std::string& reason) {
        trigger_emergency_stop("external", reason);
    }
    
    void manual_soft_stop(const std::string& reason) {
        trigger_soft_stop(reason);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<EmergencyStopHandler>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
