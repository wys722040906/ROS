/**
 * @file robot_navigator_client.cpp
 * @brief 机器人导航动作的客户端示例
 * @author ROS2 Tutorial
 */

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_interfaces_tutorial/action/navigate_to_goal.hpp"

using namespace std::chrono_literals;
namespace NavigateAction = ros2_interfaces_tutorial::action;

/**
 * @class RobotNavigatorClient
 * @brief 机器人导航动作的客户端类
 */
class RobotNavigatorClient : public rclcpp::Node {
public:
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction::NavigateToGoal>;
    
    /**
     * @brief 构造函数
     */
    RobotNavigatorClient() : Node("robot_navigator_client") {
        // 声明参数
        this->declare_parameter("robot_id", 1);
        this->declare_parameter("goal_x", 5.0);
        this->declare_parameter("goal_y", 3.0);
        this->declare_parameter("goal_z", 0.0);
        this->declare_parameter("max_speed", 1.0);
        this->declare_parameter("max_rotational_speed", 0.5);
        this->declare_parameter("use_path_planning", true);
        
        // 创建动作客户端
        client_ = rclcpp_action::create_client<NavigateAction::NavigateToGoal>(
            this, "navigate_to_goal");
            
        RCLCPP_INFO(this->get_logger(), "Robot Navigator Client 已启动");
        
        // 设置1秒后发送目标
        timer_ = this->create_wall_timer(
            1s, std::bind(&RobotNavigatorClient::sendGoal, this));
    }

private:
    /**
     * @brief 发送导航目标
     */
    void sendGoal() {
        // 取消定时器，避免重复发送
        timer_->cancel();
        
        // 检查服务器是否在线
        if (!client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "动作服务器不可用，退出客户端");
            rclcpp::shutdown();
            return;
        }
        
        // 获取参数
        int robot_id = this->get_parameter("robot_id").as_int();
        double goal_x = this->get_parameter("goal_x").as_double();
        double goal_y = this->get_parameter("goal_y").as_double();
        double goal_z = this->get_parameter("goal_z").as_double();
        float max_speed = static_cast<float>(this->get_parameter("max_speed").as_double());
        float max_rotational_speed = static_cast<float>(this->get_parameter("max_rotational_speed").as_double());
        bool use_path_planning = this->get_parameter("use_path_planning").as_bool();
        
        // 创建目标
        auto goal_msg = NavigateAction::NavigateToGoal::Goal();
        goal_msg.robot_id = robot_id;
        goal_msg.goal_x = goal_x;
        goal_msg.goal_y = goal_y;
        goal_msg.goal_z = goal_z;
        goal_msg.max_speed = max_speed;
        goal_msg.max_rotational_speed = max_rotational_speed;
        goal_msg.use_path_planning = use_path_planning;
        
        RCLCPP_INFO(this->get_logger(), "发送导航目标 - 目标位置: (%.2f, %.2f, %.2f), 机器人ID: %d",
            goal_msg.goal_x, goal_msg.goal_y, goal_msg.goal_z, goal_msg.robot_id);
            
        // 设置回调函数
        auto send_goal_options = rclcpp_action::Client<NavigateAction::NavigateToGoal>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = 
            std::bind(&RobotNavigatorClient::goalResponseCallback, this, std::placeholders::_1);
            
        send_goal_options.feedback_callback =
            std::bind(&RobotNavigatorClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
            
        send_goal_options.result_callback =
            std::bind(&RobotNavigatorClient::resultCallback, this, std::placeholders::_1);
            
        // 发送目标
        client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    /**
     * @brief 目标响应回调函数
     * @param future 目标句柄的未来对象
     */
    void goalResponseCallback(const GoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被服务器拒绝");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "目标被接受，开始导航");
            goal_handle_ = goal_handle;
            
            // 5秒后尝试取消导航
            cancel_timer_ = this->create_wall_timer(
                10s, std::bind(&RobotNavigatorClient::cancelGoal, this));
        }
    }
    
    /**
     * @brief 反馈回调函数
     * @param goal_handle 目标句柄
     * @param feedback 反馈消息
     */
    void feedbackCallback(
        GoalHandle::SharedPtr,
        const std::shared_ptr<const NavigateAction::NavigateToGoal::Feedback> feedback) {
            
        std::stringstream ss;
        ss << "收到导航反馈:" << std::endl
           << "  当前位置: (" << feedback->current_x << ", " 
                             << feedback->current_y << ", " 
                             << feedback->current_z << ")" << std::endl
           << "  剩余距离: " << feedback->distance_to_goal << " 米" << std::endl
           << "  当前速度: " << feedback->current_speed << " 米/秒" << std::endl
           << "  剩余时间: " << feedback->estimated_time_remaining << " 秒" << std::endl
           << "  进度: " << feedback->progress * 100.0 << "%" << std::endl
           << "  状态: " << feedback->current_status;
           
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
    
    /**
     * @brief 结果回调函数
     * @param result 结果消息
     */
    void resultCallback(const GoalHandle::WrappedResult& result) {
        // 取消可能的取消计时器
        if (cancel_timer_) {
            cancel_timer_->cancel();
        }
        
        // 处理结果
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "导航成功完成");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "导航被中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "导航被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知的结果代码");
                break;
        }
        
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            std::stringstream ss;
            ss << "导航结果:" << std::endl
               << "  成功: " << (result.result->success ? "是" : "否") << std::endl
               << "  最终位置: (" << result.result->final_x << ", " 
                                  << result.result->final_y << ", " 
                                  << result.result->final_z << ")" << std::endl
               << "  规划点数: " << result.result->path_points_count << std::endl
               << "  总距离: " << result.result->total_distance << " 米" << std::endl
               << "  总耗时: " << result.result->total_time << " 秒" << std::endl
               << "  结果消息: " << result.result->result_message;
               
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }
        
        rclcpp::shutdown();
    }
    
    /**
     * @brief 取消导航目标
     */
    void cancelGoal() {
        // 取消计时器，避免重复取消
        cancel_timer_->cancel();
        
        if (!goal_handle_) {
            RCLCPP_ERROR(this->get_logger(), "未找到要取消的目标");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "尝试取消导航目标");
        
        // 发送取消请求
        auto future = client_->async_cancel_goal(goal_handle_);
        
        // 等待取消完成
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
                
            RCLCPP_ERROR(this->get_logger(), "取消目标失败");
        } else {
            RCLCPP_INFO(this->get_logger(), "取消请求已发送");
        }
    }
    
    // 成员变量
    rclcpp_action::Client<NavigateAction::NavigateToGoal>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr cancel_timer_;
    GoalHandle::SharedPtr goal_handle_;
};

/**
 * @brief 主函数
 */
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNavigatorClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 