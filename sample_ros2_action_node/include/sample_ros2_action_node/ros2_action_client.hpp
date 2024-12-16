#ifndef __SAMPLE_ROS2_ACTION_NODE__ROS2_ACTION_CLIENT_HPP__
#define __SAMPLE_ROS2_ACTION_NODE__ROS2_ACTION_CLIENT_HPP__

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "sample_ros2_action_msgs/action/sample_action.hpp"

namespace sample_ros2_action_node
{

class Ros2ActionClient : public rclcpp::Node
{
private:
  using ActionT = sample_ros2_action_msgs::action::SampleAction;
  using ActionTGoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr action_request_timer_;
  rclcpp::TimerBase::SharedPtr action_request_timeout_timer_;

  ActionTGoalHandle::SharedPtr goal_handle_;

public:
  Ros2ActionClient() = delete;
  Ros2ActionClient(const rclcpp::NodeOptions & options);
  ~Ros2ActionClient();

  void sendRequest();
  void responseCallback(ActionTGoalHandle::SharedPtr);
  void feedbackCallback(
    ActionTGoalHandle::SharedPtr,
    const std::shared_ptr<const ActionT::Feedback>);
  void resultCallback(const ActionTGoalHandle::WrappedResult &);

private:
  void startActionRequestTimer();
};

}  // namespace sample_ros2_action_node

#endif
