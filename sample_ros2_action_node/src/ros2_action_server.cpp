#include <sample_ros2_action_node/ros2_action_server.hpp>

namespace sample_ros2_action_node
{

Ros2ActionServer::Ros2ActionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("ros2_action_server", options)
{
  this->action_server_ = rclcpp_action::create_server<ActionT>(
    this,
    "sample_action",
    std::bind(&Ros2ActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Ros2ActionServer::handleCancel, this, std::placeholders::_1),
    std::bind(&Ros2ActionServer::handleAccepted, this, std::placeholders::_1));

}

Ros2ActionServer::~Ros2ActionServer()
{
}

rclcpp_action::GoalResponse Ros2ActionServer::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ActionT::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;

  if (this->action_response_timer_) {
    RCLCPP_INFO(this->get_logger(), "Already executing an action, rejecting new goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Ros2ActionServer::handleCancel(
  const std::shared_ptr<ActionTGoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  this->action_response_timer_.reset();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Ros2ActionServer::handleAccepted(const std::shared_ptr<ActionTGoalHandle> goal_handle)
{
  (void)goal_handle;
  this->action_response_counter_ = 0;
  this->action_response_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this, goal_handle]() -> void {
      auto feedback = std::make_shared<ActionT::Feedback>();
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Feedback sent");
      this->action_response_counter_++;

      if (this->action_response_counter_ >= 10) {
        goal_handle->succeed(std::make_shared<ActionT::Result>());
        this->action_response_timer_.reset();
      }
    });
}

}  // namespace sample_ros2_action_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sample_ros2_action_node::Ros2ActionServer)
