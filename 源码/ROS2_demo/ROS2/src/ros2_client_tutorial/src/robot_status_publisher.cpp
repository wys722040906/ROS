/**
 * @file robot_status_publisher.cpp
 * @brief 发布机器人状态的节点
 * @author ROS2 Tutorial
 */

#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_interfaces_tutorial/msg/robot_status.hpp"

using namespace std::chrono_literals;

/**
 * @class RobotStatusPublisher
 * @brief 发布机器人状态的节点类
 */
class RobotStatusPublisher : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    RobotStatusPublisher() : Node("robot_status_publisher") {
        // 声明并获取参数
        this->declare_parameter("robot_id", 1);
        this->declare_parameter("robot_name", "tutorial_robot");
        this->declare_parameter("publish_freq", 1.0);  // Hz
        
        // 获取参数值
        robot_id_ = this->get_parameter("robot_id").as_int();
        robot_name_ = this->get_parameter("robot_name").as_string();
        double publish_freq = this->get_parameter("publish_freq").as_double();
        
        // 创建随机数生成器
        std::random_device rd;
        gen_ = std::mt19937(rd());
        battery_dist_ = std::uniform_real_distribution<float>(20.0, 100.0);
        position_dist_ = std::uniform_real_distribution<double>(-10.0, 10.0);
        orientation_dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
        error_dist_ = std::uniform_int_distribution<int>(0, 10);
        
        // 创建发布者
        publisher_ = this->create_publisher<ros2_interfaces_tutorial::msg::RobotStatus>(
            "robot_status", 10);
            
        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_freq)),
            std::bind(&RobotStatusPublisher::publishStatus, this));
            
        RCLCPP_INFO(this->get_logger(), "Robot Status Publisher 已启动 (ID: %d, 名称: %s)",
            robot_id_, robot_name_.c_str());
    }

private:
    /**
     * @brief 定时发布机器人状态
     */
    void publishStatus() {
        auto message = ros2_interfaces_tutorial::msg::RobotStatus();
        
        // 设置时间戳
        message.stamp = this->now();
        
        // 设置基本信息
        message.robot_id = robot_id_;
        message.robot_name = robot_name_;
        
        // 随机生成状态信息（在实际应用中应该从硬件读取）
        message.battery_percentage = battery_dist_(gen_);
        
        // 如果电池电量过低，设置为"错误"状态
        if (message.battery_percentage < 30.0) {
            message.status = 2;  // 错误
        } else if (message.battery_percentage < 50.0) {
            message.status = 1;  // 忙碌
        } else {
            message.status = 0;  // 空闲
        }
        
        // 随机生成位置和方向（在实际应用中应该从定位系统获取）
        message.position_x = position_dist_(gen_);
        message.position_y = position_dist_(gen_);
        message.position_z = position_dist_(gen_);
        
        // 确保四元数单位长度
        double qx = orientation_dist_(gen_);
        double qy = orientation_dist_(gen_);
        double qz = orientation_dist_(gen_);
        double qw = orientation_dist_(gen_);
        double norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        
        message.orientation_x = qx / norm;
        message.orientation_y = qy / norm;
        message.orientation_z = qz / norm;
        message.orientation_w = qw / norm;
        
        // 随机生成错误情况
        int error_val = error_dist_(gen_);
        if (error_val == 0 || message.battery_percentage < 30.0) {
            message.has_error = true;
            if (message.battery_percentage < 30.0) {
                message.error_message = "电池电量低，请充电！";
            } else {
                message.error_message = "机器人传感器故障，需要维护！";
            }
        } else {
            message.has_error = false;
            message.error_message = "";
        }
        
        // 发布消息
        publisher_->publish(message);
        
        RCLCPP_DEBUG(this->get_logger(), "已发布机器人状态 (ID: %d, 电池: %.1f%%)",
            message.robot_id, message.battery_percentage);
            
        // 如果有错误，记录警告信息
        if (message.has_error) {
            RCLCPP_WARN(this->get_logger(), "机器人出现错误: %s", message.error_message.c_str());
        }
    }

    // 成员变量
    int robot_id_;                                  // 机器人ID
    std::string robot_name_;                        // 机器人名称
    
    // 随机数生成器
    std::mt19937 gen_;
    std::uniform_real_distribution<float> battery_dist_;
    std::uniform_real_distribution<double> position_dist_;
    std::uniform_real_distribution<double> orientation_dist_;
    std::uniform_int_distribution<int> error_dist_;
    
    // ROS2发布者和定时器
    rclcpp::Publisher<ros2_interfaces_tutorial::msg::RobotStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief 主函数
 */
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 