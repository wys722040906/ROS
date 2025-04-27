/**
 * @file parameter_client.cpp
 * @brief Example of a ROS2 parameter client
 * @author ROS2 Tutorial
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * @class ParameterClient
 * @brief Demonstrates parameter client functionality in ROS2
 * 
 * This class shows how to get, set and list parameters on a remote node
 * using the parameter service client.
 */
class ParameterClient : public rclcpp::Node {
public:
    ParameterClient() : Node("parameter_client") {
        // Create parameter clients for a remote node
        // The first parameter is the remote node name
        parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(
            this, "parameter_demo_node");
            
        // Wait for the remote node's services to be available
        while (!parameters_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for parameter service...");
        }
        
        // Create a timer to perform parameter operations
        timer_ = this->create_wall_timer(
            5s, std::bind(&ParameterClient::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Parameter client initialized");
    }

private:
    /**
     * @brief Timer callback that demonstrates parameter operations
     */
    void timerCallback() {
        // Cancel the timer to run only once
        timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "Starting parameter operations...");
        
        // Get current parameter values
        getAndDisplayParameters();
        
        // Change parameter values
        modifyParameters();
        
        // Get updated parameter values
        getAndDisplayParameters();
    }
    
    /**
     * @brief Get and display remote node's parameters
     */
    void getAndDisplayParameters() {
        RCLCPP_INFO(this->get_logger(), "Getting parameters from remote node:");
        
        // Get a list of all available parameters
        auto parameter_names = parameters_client_->list_parameters({}, 1);
        
        if (parameter_names.names.empty()) {
            RCLCPP_WARN(this->get_logger(), "No parameters found on remote node.");
            return;
        }
        
        // Get individual parameters
        for (const auto& name : parameter_names.names) {
            try {
                auto parameters = parameters_client_->get_parameters({name});
                if (!parameters.empty()) {
                    RCLCPP_INFO(this->get_logger(), "  %s: %s", 
                        name.c_str(), parameters[0].value_to_string().c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get parameter '%s': %s", 
                    name.c_str(), e.what());
            }
        }
        
        // Alternative: get multiple parameters at once
        auto parameters = parameters_client_->get_parameters({"string_param", "int_param"});
        if (parameters.size() == 2) {
            RCLCPP_INFO(this->get_logger(), "Got multiple parameters at once:");
            RCLCPP_INFO(this->get_logger(), "  string_param: %s", 
                parameters[0].value_to_string().c_str());
            RCLCPP_INFO(this->get_logger(), "  int_param: %s", 
                parameters[1].value_to_string().c_str());
        }
    }
    
    /**
     * @brief Modify remote node's parameters
     */
    void modifyParameters() {
        RCLCPP_INFO(this->get_logger(), "Modifying parameters on remote node:");
        
        // 使用set_parameters而不是set_parameter来设置单个参数
        std::vector<rclcpp::Parameter> single_param;
        single_param.push_back(rclcpp::Parameter("string_param", "Updated by client"));
        auto single_result = parameters_client_->set_parameters(single_param);
        
        // 输出单个参数设置的结果
        if (!single_result.empty() && single_result[0].successful) {
            RCLCPP_INFO(this->get_logger(), "Successfully set parameter 'string_param'");
        } else if (!single_result.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameter 'string_param': %s",
                single_result[0].reason.c_str());
        }
        
        // Use set_parameters to set multiple parameters at once
        std::vector<rclcpp::Parameter> new_parameters;
        new_parameters.push_back(rclcpp::Parameter("int_param", 100));
        new_parameters.push_back(rclcpp::Parameter("bool_param", false));
        
        auto results = parameters_client_->set_parameters(new_parameters);
        
        // Check results
        for (size_t i = 0; i < results.size(); ++i) {
            if (results[i].successful) {
                RCLCPP_INFO(this->get_logger(), "Successfully set parameter '%s'", 
                    new_parameters[i].get_name().c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameter '%s': %s", 
                    new_parameters[i].get_name().c_str(),
                    results[i].reason.c_str());
            }
        }
        
        // Example using set_parameters_atomically - either all succeed or all fail
        auto atomic_result = parameters_client_->set_parameters_atomically(new_parameters);
        if (atomic_result.successful) {
            RCLCPP_INFO(this->get_logger(), "Successfully set all parameters atomically");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameters atomically: %s", 
                atomic_result.reason.c_str());
        }
    }
    
    // Member variables
    std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Main function
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 