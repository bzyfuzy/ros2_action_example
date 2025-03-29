#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <thread>
#include "ros2_action_example/action/example.hpp"

using ExampleAction = ros2_action_example::action::Example;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExampleAction>;

class ActionServer : public rclcpp::Node {
public:
    ActionServer() : Node("action_server") {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<ExampleAction>(
            this,
            "example_action",
            std::bind(&ActionServer::handle_goal, this, _1, _2),
            std::bind(&ActionServer::handle_cancel, this, _1),
            std::bind(&ActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::Server<ExampleAction>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ExampleAction::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread(std::bind(&ActionServer::execute, this, std::placeholders::_1), goal_handle).detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto result = std::make_shared<ExampleAction::Result>();
        auto start_time = this->now();

        while ((this->now() - start_time).seconds() < 5.0) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
        } else {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}