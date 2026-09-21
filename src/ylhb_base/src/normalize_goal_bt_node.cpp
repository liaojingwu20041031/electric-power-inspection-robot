#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace ylhb_base
{

class NormalizeGoal : public BT::SyncActionNode
{
public:
  NormalizeGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    config.blackboard->get("node", node_);
    config.blackboard->get("tf_buffer", tf_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Goal from NavigateToPose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal", "Goal normalized to map"),
      BT::InputPort<std::string>("global_frame", "map", "Frame used by the global costmap"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped input_goal;
    if (!getInput("input_goal", input_goal)) {
      RCLCPP_WARN(node_->get_logger(), "NormalizeGoal missing input_goal");
      return BT::NodeStatus::FAILURE;
    }

    getInput("global_frame", global_frame_);

    if (goal_cached_ && sameGoal(input_goal, last_input_goal_)) {
      setOutput("output_goal", cached_map_goal_);
      return BT::NodeStatus::SUCCESS;
    }

    if (input_goal.header.frame_id.empty()) {
      RCLCPP_WARN(node_->get_logger(), "NormalizeGoal received goal with empty frame_id");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped map_goal;
    if (input_goal.header.frame_id == global_frame_) {
      map_goal = input_goal;
    } else if (!transformGoal(input_goal, map_goal)) {
      return BT::NodeStatus::FAILURE;
    }

    // The goal is now fixed in map. Use latest-time semantics for later replans so
    // a long navigation task cannot age out of the TF cache.
    map_goal.header.frame_id = global_frame_;
    map_goal.header.stamp.sec = 0;
    map_goal.header.stamp.nanosec = 0;

    last_input_goal_ = input_goal;
    cached_map_goal_ = map_goal;
    goal_cached_ = true;
    setOutput("output_goal", cached_map_goal_);
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool transformGoal(
    const geometry_msgs::msg::PoseStamped & input_goal,
    geometry_msgs::msg::PoseStamped & map_goal)
  {
    try {
      map_goal = tf_->transform(input_goal, global_frame_, tf2::durationFromSec(0.2));
      return true;
    } catch (const tf2::TransformException & exact_error) {
      geometry_msgs::msg::PoseStamped latest_goal = input_goal;
      latest_goal.header.stamp.sec = 0;
      latest_goal.header.stamp.nanosec = 0;
      try {
        map_goal = tf_->transform(latest_goal, global_frame_, tf2::durationFromSec(0.2));
        RCLCPP_WARN(
          node_->get_logger(),
          "NormalizeGoal transformed goal from %s to %s using latest TF after exact-time TF failed: %s",
          input_goal.header.frame_id.c_str(), global_frame_.c_str(), exact_error.what());
        return true;
      } catch (const tf2::TransformException & latest_error) {
        RCLCPP_WARN(
          node_->get_logger(),
          "NormalizeGoal failed to transform goal from %s to %s: %s",
          input_goal.header.frame_id.c_str(), global_frame_.c_str(), latest_error.what());
        return false;
      }
    }
  }

  static bool sameGoal(
    const geometry_msgs::msg::PoseStamped & lhs,
    const geometry_msgs::msg::PoseStamped & rhs)
  {
    constexpr double eps = 1e-9;
    return lhs.header.frame_id == rhs.header.frame_id &&
      lhs.header.stamp.sec == rhs.header.stamp.sec &&
      lhs.header.stamp.nanosec == rhs.header.stamp.nanosec &&
      std::abs(lhs.pose.position.x - rhs.pose.position.x) < eps &&
      std::abs(lhs.pose.position.y - rhs.pose.position.y) < eps &&
      std::abs(lhs.pose.position.z - rhs.pose.position.z) < eps &&
      std::abs(lhs.pose.orientation.x - rhs.pose.orientation.x) < eps &&
      std::abs(lhs.pose.orientation.y - rhs.pose.orientation.y) < eps &&
      std::abs(lhs.pose.orientation.z - rhs.pose.orientation.z) < eps &&
      std::abs(lhs.pose.orientation.w - rhs.pose.orientation.w) < eps;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string global_frame_{"map"};
  bool goal_cached_{false};
  geometry_msgs::msg::PoseStamped last_input_goal_;
  geometry_msgs::msg::PoseStamped cached_map_goal_;
};

}  // namespace ylhb_base

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ylhb_base::NormalizeGoal>("NormalizeGoal");
}
