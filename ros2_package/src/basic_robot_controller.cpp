#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class BasicRobotController : public rclcpp::Node
{
public:
    BasicRobotController() : Node("basic_robot_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Basic Robot Controller Node Started");
        // TODO: Implement TCP robot controller
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BasicRobotController>());
    rclcpp::shutdown();
    return 0;
}
