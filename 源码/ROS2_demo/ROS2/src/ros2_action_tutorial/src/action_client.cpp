/**
 * @file action_client.cpp
 * @brief Example of a ROS2 action client
 * @author ROS2 Tutorial
 */

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

/**
 * @class JointTrajectoryActionClient
 * @brief Demonstrates action client functionality in ROS2
 * 
 * This class shows how to create a node that calls an action server
 * to execute a joint trajectory with progress feedback.
 */
class JointTrajectoryActionClient : public rclcpp::Node {
public:
    JointTrajectoryActionClient() : Node("joint_trajectory_action_client") {
        // Create an action client
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "follow_joint_trajectory");
        
        RCLCPP_INFO(this->get_logger(), "Joint trajectory action client initialized");
        
        // Wait for the action server and send a goal
        this->timer_ = this->create_wall_timer(
            500ms, std::bind(&JointTrajectoryActionClient::sendGoal, this));
    }

private:
    /**
     * @brief Send a goal to the action server
     */
    void sendGoal() {
        // Cancel the timer to ensure we only send the goal once
        this->timer_->cancel();

        // Wait for the action server to be available
        if (!action_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        // Create a goal message
        auto goal_msg = FollowJointTrajectory::Goal();

        // Define joint names (would be robot-specific in a real application)
        goal_msg.trajectory.joint_names.push_back("joint1");
        goal_msg.trajectory.joint_names.push_back("joint2");
        
        // Define trajectory points with timestamps
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // First point - initial position
        point.positions = {0.0, 0.0};
        point.velocities = {0.0, 0.0};
        point.accelerations = {0.0, 0.0};
        point.time_from_start = rclcpp::Duration(0s);
        goal_msg.trajectory.points.push_back(point);
        
        // Second point - intermediate position
        point.positions = {0.5, 0.5};
        point.velocities = {0.1, 0.1};
        point.accelerations = {0.1, 0.1};
        point.time_from_start = rclcpp::Duration(2s);
        goal_msg.trajectory.points.push_back(point);
        
        // Third point - final position
        point.positions = {1.0, 1.0};
        point.velocities = {0.0, 0.0};
        point.accelerations = {0.0, 0.0};
        point.time_from_start = rclcpp::Duration(4s);
        goal_msg.trajectory.points.push_back(point);
        
        // Set tolerances
        goal_msg.goal_tolerance.resize(2);
        goal_msg.goal_tolerance[0].name = "joint1";
        goal_msg.goal_tolerance[0].position = 0.01;
        goal_msg.goal_tolerance[1].name = "joint2";
        goal_msg.goal_tolerance[1].position = 0.01;
        
        goal_msg.goal_time_tolerance = rclcpp::Duration(1s);
        
        RCLCPP_INFO(this->get_logger(), "Sending goal with %zu trajectory points", 
                    goal_msg.trajectory.points.size());
        
        // Send the goal and set up callback functions
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        // Callback for goal response
        send_goal_options.goal_response_callback = 
            std::bind(&JointTrajectoryActionClient::goalResponseCallback, this, std::placeholders::_1);
        
        // Callback for feedback
        send_goal_options.feedback_callback = 
            std::bind(&JointTrajectoryActionClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        
        // Callback for result
        send_goal_options.result_callback = 
            std::bind(&JointTrajectoryActionClient::resultCallback, this, std::placeholders::_1);
        
        // Send the goal
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * @brief Process the goal response
     * @param future Future containing the goal handle
     */
    void goalResponseCallback(const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        
        // After 3 seconds, demonstrate how to cancel a goal
        auto cancel_timer = this->create_wall_timer(
            3s, [this, goal_handle]() {
                // Uncomment the following lines to test goal cancellation
                // RCLCPP_INFO(this->get_logger(), "Canceling goal");
                // action_client_->async_cancel_goal(goal_handle);
                
                // Remove the timer after execution
                rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                this->cancel_timer_ = nullptr;
            });
        
        this->cancel_timer_ = cancel_timer;
    }

    /**
     * @brief Process feedback from the action server
     * @param goal_handle Goal handle
     * @param feedback Feedback message
     */
    void feedbackCallback(
        GoalHandleFollowJointTrajectory::SharedPtr,
        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
        // Display progress information
        RCLCPP_INFO(this->get_logger(), "Received feedback: Processing trajectory");
        
        // Show the joint positions
        std::stringstream ss;
        ss << "Current positions: [";
        for (size_t i = 0; i < feedback->actual.positions.size(); ++i) {
            ss << feedback->actual.positions[i];
            if (i < feedback->actual.positions.size() - 1) {
                ss << ", ";
            }
        }
        ss << "]";
        
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    /**
     * @brief Process the result of the action
     * @param result Result message
     */
    void resultCallback(const GoalHandleFollowJointTrajectory::WrappedResult& result) {
        // Process the final result
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Result: error_code=%d", result.result->error_code);
        }
        
        rclcpp::shutdown();
    }

    // Member variables
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr cancel_timer_;
};

/**
 * @brief Main function
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointTrajectoryActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 