/**
 * @file robot_move_server.cpp
 * @brief 提供机器人移动服务的节点
 * @author ROS2 Tutorial
 */

#include <chrono>
#include <memory>
#include <thread>
#include <cmath>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_interfaces_tutorial/srv/move_robot.hpp"

using namespace std::chrono_literals;

/**
 * @class RobotMoveServer
 * @brief 提供机器人移动服务的节点类
 */
class RobotMoveServer : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    RobotMoveServer() : Node("robot_move_server") {
        // 创建服务
        server_ = this->create_service<ros2_interfaces_tutorial::srv::MoveRobot>(
            "move_robot",
            std::bind(&RobotMoveServer::handleMoveRequest, this,
                std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Robot Move Server 已启动，等待客户端请求...");
    }

private:
    /**
     * @brief 处理移动请求
     * @param request 请求数据
     * @param response 响应数据
     */
    void handleMoveRequest(
        const std::shared_ptr<ros2_interfaces_tutorial::srv::MoveRobot::Request> request,
        std::shared_ptr<ros2_interfaces_tutorial::srv::MoveRobot::Response> response) {
            
        RCLCPP_INFO(this->get_logger(), "收到移动请求 - 机器人ID: %d, 目标: (%.2f, %.2f, %.2f), 速度: %.2f",
            request->robot_id, request->target_x, request->target_y, request->target_z, request->speed_factor);
            
        if (request->robot_id <= 0) {
            response->success = false;
            response->message = "无效的机器人ID";
            response->elapsed_time = 0.0;
            response->actual_x = 0.0;
            response->actual_y = 0.0;
            response->actual_z = 0.0;
            return;
        }
        
        if (request->speed_factor <= 0.0 || request->speed_factor > 1.0) {
            response->success = false;
            response->message = "无效的速度因子（应在0.0-1.0范围内）";
            response->elapsed_time = 0.0;
            response->actual_x = 0.0;
            response->actual_y = 0.0;
            response->actual_z = 0.0;
            return;
        }
        
        // 计算运动距离
        double distance = std::sqrt(
            std::pow(request->target_x, 2) +
            std::pow(request->target_y, 2) +
            std::pow(request->target_z, 2));
            
        // 计算移动时间
        double base_speed = 1.0; // 米/秒
        double adjusted_speed = base_speed * request->speed_factor;
        double move_time = distance / adjusted_speed;
        
        RCLCPP_INFO(this->get_logger(), "机器人移动 - 距离: %.2f米, 速度: %.2f米/秒, 预计时间: %.2f秒",
            distance, adjusted_speed, move_time);
        
        // 模拟移动（在实际应用中这里应该发送控制命令到机器人）
        auto start_time = std::chrono::steady_clock::now();
        
        // 如果是长时间移动，模拟一段时间以避免阻塞服务
        if (move_time > 2.0) {
            // 模拟较短的时间
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(2000)));
            move_time = 2.0; // 修改实际耗时
        } else {
            // 模拟实际移动时间
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(move_time * 1000)));
        }
        
        auto end_time = std::chrono::steady_clock::now();
        double actual_time = std::chrono::duration<double>(end_time - start_time).count();
        
        // 模拟移动可能有些偏差
        double error_factor = 0.05; // 5%误差
        double x_error = request->target_x * error_factor * (static_cast<double>(rand()) / RAND_MAX - 0.5);
        double y_error = request->target_y * error_factor * (static_cast<double>(rand()) / RAND_MAX - 0.5);
        double z_error = request->target_z * error_factor * (static_cast<double>(rand()) / RAND_MAX - 0.5);
        
        // 设置响应
        response->success = true;
        response->actual_x = request->target_x + x_error;
        response->actual_y = request->target_y + y_error;
        response->actual_z = request->target_z + z_error;
        response->elapsed_time = actual_time;
        response->message = "机器人成功移动到目标位置附近";
        
        RCLCPP_INFO(this->get_logger(), "机器人移动完成 - 实际位置: (%.2f, %.2f, %.2f), 耗时: %.2f秒",
            response->actual_x, response->actual_y, response->actual_z, response->elapsed_time);
    }

    // 服务器
    rclcpp::Service<ros2_interfaces_tutorial::srv::MoveRobot>::SharedPtr server_;
};

/**
 * @brief 主函数
 */
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotMoveServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 