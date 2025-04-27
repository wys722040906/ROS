/**
 * @file service_server.cpp
 * @brief Example of a ROS2 service server
 * @author ROS2 Tutorial
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

/**
 * @class AdditionServiceNode
 * @brief Demonstrates service server functionality in ROS2
 * 
 * This class shows how to create a node that provides a service
 * that can be called by clients.
 */
class AdditionServiceNode : public rclcpp::Node {
public:
    AdditionServiceNode() : Node("addition_service_server") {
        // Create a service with a service type and a callback function
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",  // Service name
            std::bind(&AdditionServiceNode::handleAdditionRequest, this, 
                      std::placeholders::_1, std::placeholders::_2)
        );
        
        RCLCPP_INFO(this->get_logger(), "Addition service server started");
    }

private:
    /**
     * @brief Service callback function to handle addition requests
     * @param request The request containing two integers
     * @param response The response with the sum
     */
    void handleAdditionRequest(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        // Process the request and fill in the response
        response->sum = request->a + request->b;
        
        RCLCPP_INFO(this->get_logger(), 
                    "Request: a=%ld, b=%ld", request->a, request->b);
        RCLCPP_INFO(this->get_logger(), 
                    "Response: sum=%ld", response->sum);
    }
    
    // Service member
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

/**
 * @brief Main function
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdditionServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 