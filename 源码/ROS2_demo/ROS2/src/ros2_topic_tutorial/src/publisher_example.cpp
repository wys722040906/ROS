/**
 * @file publisher_example.cpp
 * @brief Example of a ROS2 publisher node
 * @author ROS2 Tutorial
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @class MinimalPublisher
 * @brief Demonstrates basic publisher functionality in ROS2
 * 
 * This class shows how to create a node that publishes messages on a topic
 * at a fixed rate using a timer.
 */
class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0) {
        // Create a publisher with a topic name and QoS settings
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        
        // Create a timer that triggers the callback at the specified interval
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timerCallback, this));
        
        // Demonstrate parameter usage
        this->declare_parameter("message_prefix", "Hello");
        
        RCLCPP_INFO(this->get_logger(), "Publisher node initialized");
    }

private:
    /**
     * @brief Timer callback function that publishes a message
     */
    void timerCallback() {
        // Get parameter value
        std::string prefix = this->get_parameter("message_prefix").as_string();
        
        // Create message
        auto message = std_msgs::msg::String();
        message.data = prefix + " World: " + std::to_string(count_++);
        
        // Publish the message
        publisher_->publish(message);
        
        // Log the published message
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    // Class members
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

/**
 * @brief Main function
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 