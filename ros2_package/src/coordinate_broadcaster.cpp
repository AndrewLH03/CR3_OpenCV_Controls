/**
 * Coordinate Broadcaster for CR3 Robot System
 * Publishes TF2 coordinate frame transformations
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class CoordinateBroadcaster : public rclcpp::Node
{
public:
    CoordinateBroadcaster() : Node("coordinate_broadcaster")
    {
        // Initialize transform broadcasters
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        
        // Declare parameters
        declare_calibration_parameters();
        load_calibration_config();
        
        // Publish static transforms
        publish_static_transforms();
        
        // Create timer for dynamic transforms (if needed)
        transform_timer_ = create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 Hz
            std::bind(&CoordinateBroadcaster::publish_dynamic_transforms, this)
        );
        
        RCLCPP_INFO(get_logger(), "Coordinate Broadcaster initialized");
    }

private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr transform_timer_;
    
    // Frame names
    std::string world_frame_;
    std::string robot_base_frame_;
    std::string camera_frame_;
    
    // Camera calibration transform
    geometry_msgs::msg::TransformStamped camera_transform_;
    
    void declare_calibration_parameters()
    {
        declare_parameter("calibration.world_frame", "world");
        declare_parameter("calibration.robot_base_frame", "base_link");
        declare_parameter("calibration.camera_frame", "camera_link");
        
        // Camera transform parameters (will be set during calibration)
        declare_parameter("calibration.camera.translation", std::vector<double>{0.0, 0.0, 500.0});
        declare_parameter("calibration.camera.rotation", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    }
    
    void load_calibration_config()
    {
        world_frame_ = get_parameter("calibration.world_frame").as_string();
        robot_base_frame_ = get_parameter("calibration.robot_base_frame").as_string();
        camera_frame_ = get_parameter("calibration.camera_frame").as_string();
        
        // Load camera transform
        auto translation = get_parameter("calibration.camera.translation").as_double_array();
        auto rotation = get_parameter("calibration.camera.rotation").as_double_array();
        
        camera_transform_.header.frame_id = robot_base_frame_;
        camera_transform_.child_frame_id = camera_frame_;
        camera_transform_.transform.translation.x = translation[0] / 1000.0;  // Convert mm to m
        camera_transform_.transform.translation.y = translation[1] / 1000.0;
        camera_transform_.transform.translation.z = translation[2] / 1000.0;
        camera_transform_.transform.rotation.x = rotation[0];
        camera_transform_.transform.rotation.y = rotation[1];
        camera_transform_.transform.rotation.z = rotation[2];
        camera_transform_.transform.rotation.w = rotation[3];
        
        RCLCPP_INFO(get_logger(), "Loaded camera transform: [%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f, %.3f]",
                    camera_transform_.transform.translation.x,
                    camera_transform_.transform.translation.y,
                    camera_transform_.transform.translation.z,
                    camera_transform_.transform.rotation.x,
                    camera_transform_.transform.rotation.y,
                    camera_transform_.transform.rotation.z,
                    camera_transform_.transform.rotation.w);
    }
    
    void publish_static_transforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        
        // World to robot base transform (identity for now)
        geometry_msgs::msg::TransformStamped world_to_base;
        world_to_base.header.stamp = this->get_clock()->now();
        world_to_base.header.frame_id = world_frame_;
        world_to_base.child_frame_id = robot_base_frame_;
        world_to_base.transform.translation.x = 0.0;
        world_to_base.transform.translation.y = 0.0;
        world_to_base.transform.translation.z = 0.0;
        world_to_base.transform.rotation.x = 0.0;
        world_to_base.transform.rotation.y = 0.0;
        world_to_base.transform.rotation.z = 0.0;
        world_to_base.transform.rotation.w = 1.0;
        
        transforms.push_back(world_to_base);
        
        // Robot base to camera transform
        camera_transform_.header.stamp = this->get_clock()->now();
        transforms.push_back(camera_transform_);
        
        // Publish all static transforms
        static_tf_broadcaster_->sendTransform(transforms);
        
        RCLCPP_INFO(get_logger(), "Published static transforms for %s -> %s -> %s",
                    world_frame_.c_str(), robot_base_frame_.c_str(), camera_frame_.c_str());
    }
    
    void publish_dynamic_transforms()
    {
        // For now, we don't have dynamic transforms
        // This method can be used later for transforms that change over time
        // For example, robot end-effector position updates
    }
    
public:
    void update_camera_transform(const geometry_msgs::msg::TransformStamped& new_transform)
    {
        camera_transform_ = new_transform;
        camera_transform_.header.stamp = this->get_clock()->now();
        
        // Republish the updated static transform
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(camera_transform_);
        static_tf_broadcaster_->sendTransform(transforms);
        
        RCLCPP_INFO(get_logger(), "Updated camera transform");
    }
    
    void set_camera_position(double x, double y, double z, double qx, double qy, double qz, double qw)
    {
        camera_transform_.transform.translation.x = x / 1000.0;  // Convert mm to m
        camera_transform_.transform.translation.y = y / 1000.0;
        camera_transform_.transform.translation.z = z / 1000.0;
        camera_transform_.transform.rotation.x = qx;
        camera_transform_.transform.rotation.y = qy;
        camera_transform_.transform.rotation.z = qz;
        camera_transform_.transform.rotation.w = qw;
        
        // Update timestamp and republish
        camera_transform_.header.stamp = this->get_clock()->now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(camera_transform_);
        static_tf_broadcaster_->sendTransform(transforms);
        
        RCLCPP_INFO(get_logger(), "Set camera position: [%.1f, %.1f, %.1f]mm", x, y, z);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CoordinateBroadcaster>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
