#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class GhostRelayInternal : public rclcpp::Node
{
public:
  GhostRelayInternal()
  : Node("ghost_relay_internal")
  {
    // Topic names are parameters so you can remap without recompiling.
    const auto in_topic = declare_parameter<std::string>(
      "input_topic", "/display_planned_path");
    const auto out_topic = declare_parameter<std::string>(
      "output_topic", "/ghost_trajectory");

    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      out_topic, rclcpp::QoS(10).reliable());

    sub_ = create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      in_topic, rclcpp::QoS(10).reliable(),
      std::bind(&GhostRelayInternal::on_display_trajectory, this,
                std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Relaying %s -> %s",
                in_topic.c_str(), out_topic.c_str());
  }

private:
  void on_display_trajectory(
    moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr msg)
  {
    if (msg->trajectory.empty()) {
      RCLCPP_WARN(get_logger(), "DisplayTrajectory has no trajectories, skipping");
      return;
    }

    // Keep only the joint trajectory of the final plan.
    // trajectory_start (RobotState), collision objects and
    // multi_dof_joint_trajectory are intentionally dropped.
    const auto & jt = msg->trajectory.back().joint_trajectory;

    if (jt.joint_names.empty() || jt.points.empty()) {
      RCLCPP_WARN(get_logger(), "Joint trajectory is empty, skipping");
      return;
    }

    pub_->publish(jt);
    RCLCPP_INFO(get_logger(), "Relayed %zu joints, %zu points",
                jt.joint_names.size(), jt.points.size());
  }

  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GhostRelayInternal>());
  rclcpp::shutdown();
  return 0;
}