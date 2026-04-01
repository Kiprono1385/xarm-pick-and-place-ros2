/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 * Modified by Brian Kiprono
 * Changes: Added custom pick-and-place logic, updated motion planning, integrated ROS2 nodes
 */

// Modified by Brian Kiprono
// Changes: Added custom pick-and-place logic, updated motion planning, integrated ROS2 nodes

#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "xarm_planner/xarm_planner.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("test_xarm_pick_place", node_options);
    RCLCPP_INFO(node->get_logger(), "test_xarm_pick_place start");

    signal(SIGINT, SIG_DFL);

    // Determine MoveIt group names from parameters.
    int dof;
    node->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));

    std::string arm_group = robot_type;
    if (robot_type == "xarm" || robot_type == "lite")
        arm_group = robot_type + std::to_string(dof);
    if (!prefix.empty())
        arm_group = prefix + arm_group;

    std::string gripper_group = robot_type + "_gripper";
    if (!prefix.empty())
        gripper_group = prefix + gripper_group;

    RCLCPP_INFO(node->get_logger(), "namespace: %s, arm_group: %s, gripper_group: %s",
                node->get_namespace(), arm_group.c_str(), gripper_group.c_str());

    xarm_planner::XArmPlanner arm_planner(node, arm_group);
    xarm_planner::XArmPlanner gripper_planner(node, gripper_group);

    // Gripper joint targets.
    std::vector<double> gripper_open(6, 0.0);
    std::vector<double> gripper_close(6, 0.85);

    // Poses for pick/place (same orientation).
    geometry_msgs::msg::Pose approach_pick;
    approach_pick.position.x = 0.30;
    approach_pick.position.y = 0.00;
    approach_pick.position.z = 0.25;
    approach_pick.orientation.x = 1;
    approach_pick.orientation.y = 0;
    approach_pick.orientation.z = 0;
    approach_pick.orientation.w = 0;

    geometry_msgs::msg::Pose pick_pose = approach_pick;
    pick_pose.position.z = 0.15;  // keep higher to avoid joint/IK limits

    geometry_msgs::msg::Pose lift_pose = approach_pick;
    lift_pose.position.z = 0.35;

    geometry_msgs::msg::Pose approach_place = approach_pick;
    approach_place.position.x = 0.32;  // closer in X to stay in limits
    approach_place.position.y = 0.05;  // smaller lateral offset
    approach_place.position.z = 0.25;  // maintain safe height

    geometry_msgs::msg::Pose place_pose = approach_place;
    place_pose.position.z = 0.15;  // avoid deep reach

    geometry_msgs::msg::Pose retreat_pose = approach_place;
    retreat_pose.position.z = 0.30;

    auto go_pose = [&](const geometry_msgs::msg::Pose &pose) {
        if (!arm_planner.planPoseTarget(pose)) return false;
        return arm_planner.executePath();
    };

    auto set_gripper = [&](const std::vector<double> &target) {
        if (!gripper_planner.planJointTarget(target)) return false;
        return gripper_planner.executePath();
    };

    RCLCPP_INFO(node->get_logger(), "Executing single pick-and-place");

    // 1) Move above the pick, then down to pick (pose plans).
    if (!go_pose(approach_pick)) return 1;
    if (!go_pose(pick_pose)) return 1;

    // 2) Close gripper.
    if (!set_gripper(gripper_close)) return 1;

    // 3) Lift, move to place, descend (pose plans).
    if (!go_pose(lift_pose)) return 1;
    if (!go_pose(approach_place)) return 1;
    if (!go_pose(place_pose)) return 1;

    // 4) Open gripper.
    if (!set_gripper(gripper_open)) return 1;

    // 5) Retreat.
    if (!go_pose(retreat_pose)) return 1;

    RCLCPP_INFO(node->get_logger(), "test_xarm_pick_place over");
    return 0;
}
 