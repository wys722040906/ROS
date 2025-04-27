/**
 * @file action_server.cpp
 * @brief Example of a ROS2 action server
 * @author ROS2 Tutorial
 */

#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

/**
 * @class JointTrajectoryActionServer
 * @brief Demonstrates action server functionality in ROS2
 * 
 * This class shows how to create a node that provides an action server
 * for the FollowJointTrajectory action, which can execute long-running tasks
 * with feedback and cancellation support.
 */
class JointTrajectoryActionServer : public rclcpp::Node {
public:
    explicit JointTrajectoryActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("joint_trajectory_action_server", options) {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "follow_joint_trajectory",
            std::bind(&JointTrajectoryActionServer::handleGoal, this, _1, _2),
            std::bind(&JointTrajectoryActionServer::handleCancel, this, _1),
            std::bind(&JointTrajectoryActionServer::handleAccepted, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Joint trajectory action server initialized");
    }

private:
    /**
     * @brief Handle the goal request for the action
     * @param uuid Goal UUID
     * @param goal Goal request
     * @return GoalResponse Status of the goal acceptance
     */
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with %zu points", 
                    goal->trajectory.points.size());
        
        // Basic validation of the goal
        if (goal->trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: trajectory is empty");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        RCLCPP_INFO(this->get_logger(), "Accepting goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief Handle a cancellation request
     * @param goal_handle The goal handle for the request being cancelled
     * @return CancelResponse Status of the cancellation
     */
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief Handle the accepted goal and execute it
     * @param goal_handle Goal handle for the accepted goal
     */
    void handleAccepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        // Start a new thread to execute the action
        std::thread{std::bind(&JointTrajectoryActionServer::executeTrajectory, this, goal_handle)}.detach();
    }

    /**
     * @brief Execute the trajectory action
     * @param goal_handle Goal handle for the goal to execute
     */
    void executeTrajectory(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        
        // Get the goal from the goal handle
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        
        // Initialize feedback
        feedback->joint_names = goal->trajectory.joint_names;
        feedback->actual.positions.resize(goal->trajectory.joint_names.size(), 0.0);
        feedback->desired.positions.resize(goal->trajectory.joint_names.size(), 0.0);
        feedback->error.positions.resize(goal->trajectory.joint_names.size(), 0.0);
        
        // Initial rate for the action execution
        rclcpp::Rate loop_rate(10);
        
        // Track progress
        size_t total_points = goal->trajectory.points.size();
        size_t current_point = 0;
        bool success = true;

        // Execute as long as the goal is valid and ROS is running
        while (rclcpp::ok() && current_point < total_points) {
            // Check if the goal was canceled
            if (goal_handle->is_canceling()) {
                result->error_code = result->SUCCESSFUL;
                result->error_string = "Goal was canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Update feedback to simulate moving towards the next trajectory point
            if (goal->trajectory.points.size() > current_point) {
                const auto& point = goal->trajectory.points[current_point];
                
                for (size_t i = 0; i < feedback->joint_names.size(); ++i) {
                    // In real implementation, these would be the actual positions from sensors
                    feedback->actual.positions[i] = point.positions[i] * 0.9; // Simulate lag
                    feedback->desired.positions[i] = point.positions[i];
                    feedback->error.positions[i] = point.positions[i] - feedback->actual.positions[i];
                }
                
                feedback->header.stamp = this->now();
                goal_handle->publish_feedback(feedback);
                current_point++;

                // Publish progress
                RCLCPP_INFO(this->get_logger(), "Executing point %zu/%zu", current_point, total_points);
            }

            loop_rate.sleep();
        }

        // Check if the goal was completed
        if (success) {
            result->error_code = result->SUCCESSFUL;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
            result->error_code = result->PATH_TOLERANCE_VIOLATED;
            result->error_string = "Failed to execute trajectory";
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "Goal failed");
        }
    }

    // Action server member
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
};

/**
 * @brief Main function
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointTrajectoryActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 