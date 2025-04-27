/**
 * @file subscriber_example.cpp
 * @brief Example of a ROS2 subscriber node
 * @author ROS2 Tutorial
 */

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @class MinimalSubscriber
 * @brief Demonstrates basic subscriber functionality in ROS2
 * 
 * This class shows how to create a node that subscribes to messages on a topic
 * and processes them in a callback.
 */
class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        // Create a subscription with a topic name, queue size, and callback
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", // Topic name matching the publisher
            10,      // Queue size
            std::bind(&MinimalSubscriber::topicCallback, this, _1)
        );
        
        // Demonstrate parameter usage
        this->declare_parameter("process_messages", true);
        
        RCLCPP_INFO(this->get_logger(), "Subscriber node initialized");
    }

private:
    /**
     * @brief Callback function that processes received messages
     * @param msg The received message
     */
    void topicCallback(const std_msgs::msg::String::SharedPtr msg) {
        // Get parameter value to check if we should process messages
        bool process = this->get_parameter("process_messages").as_bool();
        
        if (process) {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            
            // Demonstrate how to access and parse message content
            if (msg->data.find("Hello") != std::string::npos) {
                RCLCPP_INFO(this->get_logger(), "Message contains greeting");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Message processing disabled");
        }
    }
    
    // Subscriber member
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief Main function
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 