/**
 * @file robot_move_client.cpp
 * @brief 机器人移动服务的客户端示例
 * @author ROS2 Tutorial
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_interfaces_tutorial/srv/move_robot.hpp"

using namespace std::chrono_literals;

/**
 * @class RobotMoveClient
 * @brief 机器人移动服务的客户端类
 */
class RobotMoveClient : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    RobotMoveClient() : Node("robot_move_client") {
        // 声明并获取参数
        this->declare_parameter("robot_id", 1);
        this->declare_parameter("target_x", 3.0);
        this->declare_parameter("target_y", 4.0);
        this->declare_parameter("target_z", 0.0);
        this->declare_parameter("speed_factor", 0.8);
        this->declare_parameter("avoid_obstacles", true);
        
        // 创建客户端
        client_ = this->create_client<ros2_interfaces_tutorial::srv::MoveRobot>("move_robot");
        
        // 创建定时器，延迟1秒后发送请求
        timer_ = this->create_wall_timer(
            1s, std::bind(&RobotMoveClient::sendRequest, this));
            
        RCLCPP_INFO(this->get_logger(), "Robot Move Client 已启动");
    }

private:
    /**
     * @brief 发送移动请求
     */
    void sendRequest() {
        // 取消定时器，避免重复发送
        timer_->cancel();
        
        // 如果服务不可用，等待服务
        if (!client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "服务不可用，请检查服务器是否运行");
            rclcpp::shutdown();
            return;
        }
        
        // 获取参数
        int robot_id = this->get_parameter("robot_id").as_int();
        double target_x = this->get_parameter("target_x").as_double();
        double target_y = this->get_parameter("target_y").as_double();
        double target_z = this->get_parameter("target_z").as_double();
        float speed_factor = static_cast<float>(this->get_parameter("speed_factor").as_double());
        bool avoid_obstacles = this->get_parameter("avoid_obstacles").as_bool();
        
        // 创建请求
        auto request = std::make_shared<ros2_interfaces_tutorial::srv::MoveRobot::Request>();
        request->robot_id = robot_id;
        request->target_x = target_x;
        request->target_y = target_y;
        request->target_z = target_z;
        request->target_orientation_x = 0.0;  // 简化示例，不考虑方向
        request->target_orientation_y = 0.0;
        request->target_orientation_z = 0.0;
        request->target_orientation_w = 1.0;  // 默认朝向（无旋转）
        request->speed_factor = speed_factor;
        request->avoid_obstacles = avoid_obstacles;
        
        RCLCPP_INFO(this->get_logger(), "发送移动请求 - 机器人ID: %d, 目标: (%.2f, %.2f, %.2f), 速度: %.2f",
            request->robot_id, request->target_x, request->target_y, request->target_z, request->speed_factor);
            
        // 发送异步请求
        auto future = client_->async_send_request(
            request, std::bind(&RobotMoveClient::responseCallback, this, std::placeholders::_1));
    }
    
    /**
     * @brief 响应回调函数
     * @param future 包含响应的future对象
     */
    void responseCallback(
        rclcpp::Client<ros2_interfaces_tutorial::srv::MoveRobot>::SharedFuture future) {
            
        try {
            auto response = future.get();
            
            RCLCPP_INFO(this->get_logger(), "收到移动服务响应:");
            RCLCPP_INFO(this->get_logger(), "  成功: %s", response->success ? "是" : "否");
            RCLCPP_INFO(this->get_logger(), "  实际位置: (%.2f, %.2f, %.2f)",
                response->actual_x, response->actual_y, response->actual_z);
            RCLCPP_INFO(this->get_logger(), "  耗时: %.2f秒", response->elapsed_time);
            RCLCPP_INFO(this->get_logger(), "  消息: %s", response->message.c_str());
            
            // 如果操作成功且用户希望继续操作，可以在这里添加新的请求
            // 本示例在响应后退出
            rclcpp::shutdown();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "服务调用失败: %s", e.what());
            rclcpp::shutdown();
        }
    }
    
    // 成员变量
    rclcpp::Client<ros2_interfaces_tutorial::srv::MoveRobot>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief 主函数
 */
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotMoveClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}