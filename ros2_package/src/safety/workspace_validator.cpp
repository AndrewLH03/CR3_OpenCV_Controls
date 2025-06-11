/**
 * Workspace Validator for CR3 Robot System
 * Fast C++ implementation for real-time boundary checking
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cr3_hand_control/msg/robot_status.hpp>
#include <cr3_hand_control/msg/safety_alert.hpp>
#include <cmath>

class WorkspaceValidator : public rclcpp::Node
{
public:
    WorkspaceValidator() : Node("workspace_validator")
    {
        // Declare parameters
        declare_workspace_parameters();
        
        // Load workspace configuration
        load_workspace_config();
        
        // Publishers
        workspace_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/workspace_valid", 10);
        
        // Subscribers
        robot_status_sub_ = create_subscription<cr3_hand_control::msg::RobotStatus>(
            "/robot_status", 10,
            std::bind(&WorkspaceValidator::robot_status_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Workspace Validator initialized");
    }

private:
    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr workspace_valid_pub_;
    rclcpp::Subscription<cr3_hand_control::msg::RobotStatus>::SharedPtr robot_status_sub_;
    
    // Workspace boundaries
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double buffer_zone_;
    
    void declare_workspace_parameters()
    {
        declare_parameter("safety.workspace_boundaries.x_min", -400.0);
        declare_parameter("safety.workspace_boundaries.x_max", 400.0);
        declare_parameter("safety.workspace_boundaries.y_min", -400.0);
        declare_parameter("safety.workspace_boundaries.y_max", 400.0);
        declare_parameter("safety.workspace_boundaries.z_min", 50.0);
        declare_parameter("safety.workspace_boundaries.z_max", 600.0);
        declare_parameter("safety.workspace_boundaries.buffer_zone", 20.0);
    }
    
    void load_workspace_config()
    {
        x_min_ = get_parameter("safety.workspace_boundaries.x_min").as_double();
        x_max_ = get_parameter("safety.workspace_boundaries.x_max").as_double();
        y_min_ = get_parameter("safety.workspace_boundaries.y_min").as_double();
        y_max_ = get_parameter("safety.workspace_boundaries.y_max").as_double();
        z_min_ = get_parameter("safety.workspace_boundaries.z_min").as_double();
        z_max_ = get_parameter("safety.workspace_boundaries.z_max").as_double();
        buffer_zone_ = get_parameter("safety.workspace_boundaries.buffer_zone").as_double();
        
        RCLCPP_INFO(get_logger(), "Workspace boundaries loaded: X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]",
                    x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
    }
    
    void robot_status_callback(const cr3_hand_control::msg::RobotStatus::SharedPtr msg)
    {
        if (msg->current_position.size() >= 3) {
            bool is_valid = validate_position(msg->current_position[0], 
                                             msg->current_position[1], 
                                             msg->current_position[2]);
            
            auto valid_msg = std_msgs::msg::Bool();
            valid_msg.data = is_valid;
            workspace_valid_pub_->publish(valid_msg);
            
            if (!is_valid) {
                RCLCPP_WARN(get_logger(), "Robot position outside workspace boundaries");
            }
        }
    }
    
    bool validate_position(double x, double y, double z)
    {
        // Check if position is within workspace boundaries including buffer zone
        bool x_valid = (x >= (x_min_ + buffer_zone_)) && 
                      (x <= (x_max_ - buffer_zone_));
        bool y_valid = (y >= (y_min_ + buffer_zone_)) && 
                      (y <= (y_max_ - buffer_zone_));
        bool z_valid = (z >= (z_min_ + buffer_zone_)) && 
                      (z <= (z_max_ - buffer_zone_));
        
        return x_valid && y_valid && z_valid;
    }
    
public:
    // Public method for external validation calls
    bool is_position_safe(double x, double y, double z)
    {
        return (x >= (x_min_ + buffer_zone_)) && (x <= (x_max_ - buffer_zone_)) &&
               (y >= (y_min_ + buffer_zone_)) && (y <= (y_max_ - buffer_zone_)) &&
               (z >= (z_min_ + buffer_zone_)) && (z <= (z_max_ - buffer_zone_));
    }
    
    double get_min_distance_to_boundary(double x, double y, double z)
    {
        double distances[] = {
            x - x_min_,
            x_max_ - x,
            y - y_min_,
            y_max_ - y,
            z - z_min_,
            z_max_ - z
        };
        
        double min_dist = distances[0];
        for (int i = 1; i < 6; i++) {
            if (distances[i] < min_dist) {
                min_dist = distances[i];
            }
        }
        
        return min_dist;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<WorkspaceValidator>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
