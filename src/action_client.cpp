#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include "ros2_action_example/action/example.hpp"

using ExampleAction = ros2_action_example::action::Example;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExampleAction>;
using std::placeholders::_1;
using std::placeholders::_2;

class ActionClient : public rclcpp::Node {
public:
    ActionClient() : Node("action_client") {
        subscription_ = this->create_subscription<std_msgs::msg::Empty>(
            "goal_topic", 10,
            std::bind(&ActionClient::topic_callback, this, _1));

        this->client_ = rclcpp_action::create_client<ExampleAction>(
            this, "example_action");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
    rclcpp_action::Client<ExampleAction>::SharedPtr client_;
    std::shared_ptr<GoalHandle> current_goal_handle_;
    std::mutex mutex_;

    void topic_callback(const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        std::lock_guard<std::mutex> lock(mutex_);

        if (current_goal_handle_) {
            auto future_cancel = client_->async_cancel_goal(current_goal_handle_);
            RCLCPP_INFO(this->get_logger(), "Sent cancel request for previous goal");
        }

        auto goal_msg = ExampleAction::Goal();
        auto send_goal_options = rclcpp_action::Client<ExampleAction>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ActionClient::goal_response_callback, this, _1);
        send_goal_options.result_callback =
            std::bind(&ActionClient::result_callback, this, _1);
        auto future_goal = client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(std::shared_ptr<GoalHandle> goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            std::lock_guard<std::mutex> lock(mutex_);
            current_goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
    }

    void result_callback(const GoalHandle::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Goal aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_goal_handle_ && current_goal_handle_->get_goal_id() == result.goal_id) {
            current_goal_handle_.reset();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}