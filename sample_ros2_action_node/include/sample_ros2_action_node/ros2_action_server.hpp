#ifndef __SAMPLE_ROS2_ACTION_NODE__ROS2_ACTION_SERVER_HPP__
#define __SAMPLE_ROS2_ACTION_NODE__ROS2_ACTION_SERVER_HPP__

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sample_ros2_action_msgs/action/sample_action.hpp>

namespace sample_ros2_action_node
{
class Ros2ActionServer : public rclcpp::Node
{
private:
  using ActionT = sample_ros2_action_msgs::action::SampleAction;
  using ActionTGoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  rclcpp_action::Server<ActionT>::SharedPtr action_server_;

  rclcpp::TimerBase::SharedPtr action_response_timer_;
  uint8_t action_response_counter_;

public:
  Ros2ActionServer() = delete;
  explicit Ros2ActionServer(const rclcpp::NodeOptions &);
  ~Ros2ActionServer();

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, ActionT::Goal::ConstSharedPtr);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<ActionTGoalHandle>);
  void handleAccepted(const std::shared_ptr<ActionTGoalHandle>);
};

}  // namespace sample_ros2_action_node

#endif
