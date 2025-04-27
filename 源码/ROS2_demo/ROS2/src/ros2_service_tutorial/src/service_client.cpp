/**
 * @file service_client.cpp
 * @brief Example of a ROS2 service client
 * @author ROS2 Tutorial
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

/**
 * @class AdditionClientNode
 * @brief Demonstrates service client functionality in ROS2
 * 
 * This class shows how to create a node that calls a service
 * and processes the response.
 */
class AdditionClientNode : public rclcpp::Node {
public:
    AdditionClientNode() : Node("addition_service_client") {
        // Create a client for the addition service
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        
        // Wait for the service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }
        
        // Start sending requests
        sendRequest(10, 20);
    }

private:
    /**
     * @brief Send a request to the addition service
     * @param a First integer
     * @param b Second integer
     */
    void sendRequest(int64_t a, int64_t b) {
        // Create a request
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        
        RCLCPP_INFO(this->get_logger(), "Sending request: a=%ld, b=%ld", a, b);
        
        // Send the request asynchronously
        auto futureResult = client_->async_send_request(
            request, 
            std::bind(&AdditionClientNode::handleResponse, this, std::placeholders::_1)
        );
    }
    
    /**
     * @brief Callback to handle the service response
     * @param future Future object containing the response
     */
    void handleResponse(
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) 
    {
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Result of addition: %ld", response->sum);
            
            // Send another request with different values after 2 seconds
            auto timer = this->create_wall_timer(
                2s,
                [this]() {
                    static int count = 0;
                    this->sendRequest(count * 5, count * 10);
                    count++;
                    if (count > 5) {
                        RCLCPP_INFO(this->get_logger(), "Demonstration complete");
                        rclcpp::shutdown();
                    }
                });
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
    
    // Client member
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

/**
 * @brief Main function
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdditionClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 