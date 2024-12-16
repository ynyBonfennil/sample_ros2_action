#include "sample_ros2_action_node/ros2_action_client.hpp"

namespace sample_ros2_action_node
{

Ros2ActionClient::Ros2ActionClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("ros2_action_client", options)
{
  this->action_client_ = rclcpp_action::create_client<ActionT>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "sample_action");

  this->startActionRequestTimer();
}

Ros2ActionClient::~Ros2ActionClient()
{
}

void Ros2ActionClient::sendRequest()
{
  using namespace std::placeholders;

  auto goal_msg = ActionT::Goal();
  RCLCPP_INFO(this->get_logger(), "Sending goal request");

  auto send_goal_options = rclcpp_action::Client<ActionT>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&Ros2ActionClient::responseCallback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&Ros2ActionClient::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&Ros2ActionClient::resultCallback, this, _1);

  this->action_client_->async_send_goal(goal_msg, send_goal_options);
}

void Ros2ActionClient::responseCallback(ActionTGoalHandle::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void Ros2ActionClient::feedbackCallback(
  ActionTGoalHandle::SharedPtr,
  const std::shared_ptr<const ActionT::Feedback>)
{
  RCLCPP_INFO(this->get_logger(), "Received feedback");
}

void Ros2ActionClient::resultCallback(
  const ActionTGoalHandle::WrappedResult &)
{
  RCLCPP_INFO(this->get_logger(), "Received result");
  this->startActionRequestTimer();
}

void Ros2ActionClient::startActionRequestTimer()
{
  this->action_request_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      this->action_request_timer_.reset();
      this->sendRequest();
    }
  );

}

}  // namespace sample_ros2_action_node


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sample_ros2_action_node::Ros2ActionClient)
