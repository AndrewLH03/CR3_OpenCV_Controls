#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "cr3_hand_control/msg/basic_command.hpp"
#include "cr3_hand_control/msg/robot_status.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>

class SimpleTcpClient {
private:
    int socket_fd;
    struct sockaddr_in server_addr;
    bool connected;
    
public:
    SimpleTcpClient() : socket_fd(-1), connected(false) {}
    
    ~SimpleTcpClient() {
        disconnect();
    }
    
    bool connect_to_robot(const std::string& ip, int port) {
        socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("tcp_client"), "Failed to create socket");
            return false;
        }
        
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr);
        
        if (connect(socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("tcp_client"), "Failed to connect to %s:%d", ip.c_str(), port);
            close(socket_fd);
            socket_fd = -1;
            return false;
        }
        
        connected = true;
        RCLCPP_INFO(rclcpp::get_logger("tcp_client"), "Connected to %s:%d", ip.c_str(), port);
        return true;
    }
    
    void disconnect() {
        if (socket_fd >= 0) {
            close(socket_fd);
            socket_fd = -1;
        }
        connected = false;
    }
    
    bool send_command(const std::string& command) {
        if (!connected || socket_fd < 0) {
            return false;
        }
        
        std::string full_command = command + "\n";
        ssize_t sent = send(socket_fd, full_command.c_str(), full_command.length(), 0);
        return sent == static_cast<ssize_t>(full_command.length());
    }
    
    std::string receive_response() {
        if (!connected || socket_fd < 0) {
            return "";
        }
        
        char buffer[1024];
        ssize_t received = recv(socket_fd, buffer, sizeof(buffer) - 1, 0);
        if (received > 0) {
            buffer[received] = '\0';
            return std::string(buffer);
        }
        return "";
    }
    
    bool is_connected() const {
        return connected && socket_fd >= 0;
    }
};

class BasicRobotController : public rclcpp::Node {
private:
    SimpleTcpClient tcp_client;
    rclcpp::Publisher<cr3_hand_control::msg::RobotStatus>::SharedPtr status_publisher;
    rclcpp::Subscription<cr3_hand_control::msg::BasicCommand>::SharedPtr command_subscriber;
    rclcpp::TimerBase::SharedPtr status_timer;
    
    std::string robot_ip;
    int dashboard_port;
    int realtime_port;
    bool robot_enabled;
    std::array<double, 6> current_position;
    
public:
    BasicRobotController() : Node("basic_robot_controller") {
        // Declare parameters
        this->declare_parameter("robot_ip", "192.168.1.6");
        this->declare_parameter("dashboard_port", 29999);
        this->declare_parameter("realtime_port", 30003);
        
        // Get parameters
        robot_ip = this->get_parameter("robot_ip").as_string();
        dashboard_port = this->get_parameter("dashboard_port").as_int();
        realtime_port = this->get_parameter("realtime_port").as_int();
        
        robot_enabled = false;
        current_position.fill(0.0);
        
        // Create publishers and subscribers
        status_publisher = this->create_publisher<cr3_hand_control::msg::RobotStatus>(
            "robot_status", 10);
        
        command_subscriber = this->create_subscription<cr3_hand_control::msg::BasicCommand>(
            "robot_command", 10,
            std::bind(&BasicRobotController::command_callback, this, std::placeholders::_1));
        
        // Create timer for status publishing
        status_timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&BasicRobotController::publish_status, this));
        
        // Connect to robot
        connect_to_robot();
        
        RCLCPP_INFO(this->get_logger(), "Basic Robot Controller initialized");
    }
    
private:
    void connect_to_robot() {
        if (!tcp_client.connect_to_robot(robot_ip, dashboard_port)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot at %s:%d", 
                         robot_ip.c_str(), dashboard_port);
        }
    }
    
    void command_callback(const cr3_hand_control::msg::BasicCommand::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received command: %s", msg->command_type.c_str());
        
        if (!tcp_client.is_connected()) {
            RCLCPP_WARN(this->get_logger(), "Robot not connected, attempting to reconnect...");
            connect_to_robot();
            return;
        }
        
        std::string command;
        
        if (msg->command_type == "enable") {
            command = "EnableRobot()";
            robot_enabled = true;
        } else if (msg->command_type == "disable") {
            command = "DisableRobot()";
            robot_enabled = false;
        } else if (msg->command_type == "clear_error") {
            command = "ClearError()";
        } else if (msg->command_type == "movj") {
            command = "MovJ(" + 
                      std::to_string(msg->position[0]) + "," +
                      std::to_string(msg->position[1]) + "," +
                      std::to_string(msg->position[2]) + "," +
                      std::to_string(msg->position[3]) + "," +
                      std::to_string(msg->position[4]) + "," +
                      std::to_string(msg->position[5]) + ")";
        } else if (msg->command_type == "movl") {
            command = "MovL(" + 
                      std::to_string(msg->position[0]) + "," +
                      std::to_string(msg->position[1]) + "," +
                      std::to_string(msg->position[2]) + "," +
                      std::to_string(msg->position[3]) + "," +
                      std::to_string(msg->position[4]) + "," +
                      std::to_string(msg->position[5]) + ")";
        } else if (msg->command_type == "get_pose") {
            command = "GetPose()";
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown command type: %s", msg->command_type.c_str());
            return;
        }
        
        if (tcp_client.send_command(command)) {
            RCLCPP_INFO(this->get_logger(), "Sent command: %s", command.c_str());
            
            // Get response for get_pose command
            if (msg->command_type == "get_pose") {
                std::string response = tcp_client.receive_response();
                if (!response.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Robot response: %s", response.c_str());
                    // Parse position from response (simplified)
                    // In a real implementation, you'd parse the actual position data
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s", command.c_str());
        }
    }
    
    void publish_status() {
        auto status_msg = cr3_hand_control::msg::RobotStatus();
        
        status_msg.connected = tcp_client.is_connected();
        status_msg.connection_message = tcp_client.is_connected() ? "Connected" : "Disconnected";
        status_msg.enabled = robot_enabled;
        status_msg.error_state = false;
        status_msg.error_message = "";
        status_msg.robot_ip = robot_ip;
        status_msg.dashboard_port = dashboard_port;
        
        // Copy current position
        for (size_t i = 0; i < 6; ++i) {
            status_msg.current_position[i] = current_position[i];
        }
        
        status_msg.timestamp = this->now();
        
        status_publisher->publish(status_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicRobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
