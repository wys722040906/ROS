/**
 * @file robot_navigator_server.cpp
 * @brief 提供机器人导航动作的节点
 * @author ROS2 Tutorial
 */

#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_interfaces_tutorial/action/navigate_to_goal.hpp"

using namespace std::chrono_literals;
namespace NavigateAction = ros2_interfaces_tutorial::action;

/**
 * @class RobotNavigatorServer
 * @brief 提供机器人导航动作的节点类
 */
class RobotNavigatorServer : public rclcpp::Node {
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateAction::NavigateToGoal>;
    
    /**
     * @brief 构造函数
     */
    RobotNavigatorServer() : Node("robot_navigator_server") {
        // 创建动作服务器
        action_server_ = rclcpp_action::create_server<NavigateAction::NavigateToGoal>(
            this,
            "navigate_to_goal",
            std::bind(&RobotNavigatorServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotNavigatorServer::handleCancel, this, std::placeholders::_1),
            std::bind(&RobotNavigatorServer::handleAccepted, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Robot Navigator Server 已启动，等待导航请求...");
    }

private:
    /**
     * @brief 处理客户端的目标请求
     * @return 接受、拒绝或挂起请求的结果
     */
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateAction::NavigateToGoal::Goal> goal) {
            
        RCLCPP_INFO(this->get_logger(), "收到导航请求 - 机器人ID: %d, 目标位置: (%.2f, %.2f, %.2f)",
            goal->robot_id, goal->goal_x, goal->goal_y, goal->goal_z);
            
        (void)uuid;
        
        // 进行一些基本检查
        if (goal->robot_id <= 0) {
            RCLCPP_ERROR(this->get_logger(), "拒绝导航请求：无效的机器人ID");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        if (std::isnan(goal->goal_x) || std::isnan(goal->goal_y) || std::isnan(goal->goal_z)) {
            RCLCPP_ERROR(this->get_logger(), "拒绝导航请求：目标坐标包含NaN值");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        if (goal->max_speed <= 0.0 || goal->max_rotational_speed <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "拒绝导航请求：速度参数无效");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // 接受请求
        RCLCPP_INFO(this->get_logger(), "接受导航请求");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    /**
     * @brief 处理取消请求
     * @return 接受或拒绝取消请求的结果
     */
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle> goal_handle) {
            
        RCLCPP_INFO(this->get_logger(), "收到取消导航请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    /**
     * @brief 处理已接受的目标
     */
    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
        // 创建一个新线程执行导航任务
        std::thread{std::bind(&RobotNavigatorServer::executeNavigation, this, std::placeholders::_1), goal_handle}.detach();
    }
    
    /**
     * @brief 执行导航任务
     * @param goal_handle 目标句柄
     */
    void executeNavigation(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "执行导航任务...");
        
        // 获取目标
        const auto goal = goal_handle->get_goal();
        
        // 计算距离
        double distance = std::sqrt(
            std::pow(goal->goal_x, 2) + 
            std::pow(goal->goal_y, 2) + 
            std::pow(goal->goal_z, 2));
            
        // 初始化反馈
        auto feedback = std::make_shared<NavigateAction::NavigateToGoal::Feedback>();
        feedback->current_x = 0.0;
        feedback->current_y = 0.0;
        feedback->current_z = 0.0;
        feedback->distance_to_goal = distance;
        feedback->current_speed = 0.0;
        feedback->estimated_time_remaining = distance / goal->max_speed;
        feedback->progress = 0.0;
        feedback->current_status = "初始化导航";
        
        // 初始化结果
        auto result = std::make_shared<NavigateAction::NavigateToGoal::Result>();
        
        // 生成路径点数量
        int num_points = goal->use_path_planning ? 100 : 20;
        
        // 开始时间
        auto start_time = std::chrono::steady_clock::now();
        
        // 模拟导航
        for (int i = 0; i <= num_points; ++i) {
            // 检查是否被取消
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->final_x = feedback->current_x;
                result->final_y = feedback->current_y;
                result->final_z = feedback->current_z;
                result->path_points_count = i;
                result->total_distance = distance * (static_cast<double>(i) / num_points);
                result->total_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
                result->result_message = "导航被取消";
                
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "导航任务已取消");
                return;
            }
            
            // 更新当前位置（模拟直线路径）
            feedback->current_x = goal->goal_x * (static_cast<double>(i) / num_points);
            feedback->current_y = goal->goal_y * (static_cast<double>(i) / num_points);
            feedback->current_z = goal->goal_z * (static_cast<double>(i) / num_points);
            
            // 更新反馈
            feedback->distance_to_goal = distance * (1.0 - static_cast<double>(i) / num_points);
            feedback->current_speed = goal->max_speed * (i < num_points / 10 ? 
                                      static_cast<double>(i) / (num_points / 10) : // 加速
                                      (i > 9 * num_points / 10 ? 
                                        (1.0 - static_cast<double>(i - 9 * num_points / 10) / (num_points / 10)) : // 减速
                                        1.0)); // 匀速
            feedback->progress = static_cast<float>(i) / num_points;
            feedback->estimated_time_remaining = feedback->distance_to_goal / feedback->current_speed;
            
            // 更新状态
            if (i == 0) {
                feedback->current_status = "开始导航";
            } else if (i < num_points / 3) {
                feedback->current_status = "加速阶段";
            } else if (i < 2 * num_points / 3) {
                feedback->current_status = "匀速阶段";
            } else if (i < num_points) {
                feedback->current_status = "减速阶段";
            } else {
                feedback->current_status = "到达目标";
            }
            
            // 发布反馈
            goal_handle->publish_feedback(feedback);
            RCLCPP_DEBUG(this->get_logger(), "导航进度: %.1f%%, 剩余距离: %.2f米",
                feedback->progress * 100.0, feedback->distance_to_goal);
                
            // 模拟导航时间（总时间约为10秒）
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 计算总耗时
        auto end_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
        
        // 设置结果
        result->success = true;
        // 添加一点随机误差
        double error = 0.05;
        result->final_x = goal->goal_x * (1.0 + (static_cast<double>(rand()) / RAND_MAX - 0.5) * error);
        result->final_y = goal->goal_y * (1.0 + (static_cast<double>(rand()) / RAND_MAX - 0.5) * error);
        result->final_z = goal->goal_z * (1.0 + (static_cast<double>(rand()) / RAND_MAX - 0.5) * error);
        result->path_points_count = num_points;
        result->total_distance = distance;
        result->total_time = elapsed_time;
        result->result_message = "导航成功完成";
        
        // 导航成功
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "导航任务成功完成 - 总距离: %.2f米, 总耗时: %.2f秒",
            result->total_distance, result->total_time);
    }
    
    // 动作服务器
    rclcpp_action::Server<NavigateAction::NavigateToGoal>::SharedPtr action_server_;
};

/**
 * @brief 主函数
 */
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNavigatorServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 