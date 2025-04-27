/**
 * @file parameter_node.cpp
 * @brief Example of a ROS2 node with parameter server usage
 * @author ROS2 Tutorial
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * @class ParameterNode
 * @brief Demonstrates parameter server functionality in ROS2
 * 
 * This class shows how to declare, use, and update parameters in ROS2,
 * including callbacks for parameter changes and parameter description.
 */
class ParameterNode : public rclcpp::Node {
public:
    ParameterNode() : Node("parameter_demo_node") {
        // Declare parameters with default values
        this->declare_parameter("string_param", "Hello World");
        this->declare_parameter("int_param", 42);
        this->declare_parameter("double_param", 3.14);
        this->declare_parameter("bool_param", true);
        
        // Declare an array parameter
        std::vector<double> default_array = {1.1, 2.2, 3.3};
        this->declare_parameter("array_param", default_array);
        
        // Advanced: declare a parameter with a description and constraints
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "Parameter that controls publishing rate in Hz";
        desc.read_only = false;
        
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 1.0;
        range.to_value = 50.0;
        range.step = 1.0;
        desc.floating_point_range.push_back(range);
        
        this->declare_parameter("rate", 10.0, desc);
        
        // Set up callback for parameter changes
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ParameterNode::parametersCallback, this, std::placeholders::_1));
        
        // Create a timer that calls a function periodically
        update_timer_ = this->create_wall_timer(
            500ms, std::bind(&ParameterNode::timerCallback, this));
        
        display_timer_ = this->create_wall_timer(
            1s, std::bind(&ParameterNode::displayParameters, this));
        
        RCLCPP_INFO(this->get_logger(), "Parameter node initialized");
    }

private:
    /**
     * @brief Timer callback function that updates a parameter periodically
     */
    void timerCallback() {
        // Get current value of the double parameter
        double current_value = this->get_parameter("double_param").as_double();
        
        // Update the parameter with a new value
        current_value += 0.1;
        if (current_value > 10.0) {
            current_value = 0.0;
        }
        
        this->set_parameter(rclcpp::Parameter("double_param", current_value));
    }
    
    /**
     * @brief Displays all parameters periodically
     */
    void displayParameters() {
        // Get all parameter names
        auto parameters = this->get_parameters(
            {"string_param", "int_param", "double_param", "bool_param", "array_param", "rate"});
        
        // Display parameter values
        RCLCPP_INFO(this->get_logger(), "Current parameter values:");
        for (const auto& param : parameters) {
            // Format the output depending on the parameter type
            std::string value_str;
            switch (param.get_type()) {
                case rclcpp::ParameterType::PARAMETER_STRING:
                    value_str = param.as_string();
                    break;
                case rclcpp::ParameterType::PARAMETER_INTEGER:
                    value_str = std::to_string(param.as_int());
                    break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE:
                    value_str = std::to_string(param.as_double());
                    break;
                case rclcpp::ParameterType::PARAMETER_BOOL:
                    value_str = param.as_bool() ? "true" : "false";
                    break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
                    auto array = param.as_double_array();
                    value_str = "[";
                    for (size_t i = 0; i < array.size(); ++i) {
                        value_str += std::to_string(array[i]);
                        if (i < array.size() - 1) value_str += ", ";
                    }
                    value_str += "]";
                    break;
                }
                default:
                    value_str = "unknown type";
            }
            
            RCLCPP_INFO(this->get_logger(), "  %s: %s", 
                param.get_name().c_str(), value_str.c_str());
        }
    }
    
    /**
     * @brief Callback for parameter changes
     * @param parameters List of changing parameters
     * @return Parameter validation result
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters) 
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto& param : parameters) {
            // Log parameter changes
            RCLCPP_INFO(this->get_logger(), 
                        "Parameter '%s' changed to: %s", 
                        param.get_name().c_str(),
                        param.value_to_string().c_str());
            
            // Example of parameter validation
            if (param.get_name() == "rate") {
                double rate = param.as_double();
                if (rate < 1.0 || rate > 50.0) {
                    result.successful = false;
                    result.reason = "Rate must be between 1.0 and 50.0";
                    return result;
                }
            }
        }
        
        return result;
    }
    
    // Timer members
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr display_timer_;
    
    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

/**
 * @brief Main function
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 