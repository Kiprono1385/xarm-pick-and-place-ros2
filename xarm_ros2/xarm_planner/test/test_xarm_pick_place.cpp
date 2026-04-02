/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 * Modified by Brian Kiprono
 * Changes: Converted Cartesian planning to Joint Space planning for higher reliability, 
 * updated gripper closure to use radian equivalents (49 deg), 
 * integrated ROS2 nodes.
 */

#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("test_xarm_pick_place", node_options);
    RCLCPP_INFO(node->get_logger(), "test_xarm_pick_place start using Joint Space");

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

    // --- 1. JOINT TARGET DEFINITIONS ---
    // Gripper: 0.0 is open, 0.855 is 49 degrees (closed)
    std::vector<double> gripper_open(6, 0.0);
    std::vector<double> gripper_close(6, 0.855);

    // Arm Waypoints: Replace these with your actual 7-joint radian values from RViz
    // Format: {J1, J2, J3, J4, J5, J6, J7}
    std::vector<double> approach_pick_joint  = {3.1067, 0.2269, 0.0000, 1.3090, 0.0000, 1.0297, 3.1067};
    std::vector<double> pick_pose_joint      = {3.1067, 0.4014, 0.0000, 1.1868, 0.0000, 0.7505, 3.1067};
    std::vector<double> lift_pose_joint      = {3.1067, 0.2269, 0.0000, 1.3090, 0.0000, 1.0297, 3.1067};  
    std::vector<double> approach_place_joint = {0.5411, -0.9076, -2.0944, 2.2515, -0.7679, 1.6581, -1.8675};
    std::vector<double> place_pose_joint     = {0.4014, -1.0647, -2.1817, 1.8675, -0.8901, 1.2217, -1.7279};
    std::vector<double> retreat_pose_joint   = {0.5411, -0.9076, -2.0944, 2.2515, -0.7679, 1.6581, -1.8675};

    // --- 2. LAMBDA FUNCTIONS FOR PLANNING ---
    auto go_joint = [&](const std::vector<double> &target) {
        if (!arm_planner.planJointTarget(target)) {
            RCLCPP_ERROR(node->get_logger(), "Arm planning failed for joint target!");
            return false;
        }
        return arm_planner.executePath();
    };

    auto set_gripper = [&](const std::vector<double> &target) {
        if (!gripper_planner.planJointTarget(target)) {
            RCLCPP_ERROR(node->get_logger(), "Gripper planning failed!");
            return false;
        }
        return gripper_planner.executePath();
    };

    // --- 3. EXECUTION SEQUENCE ---
    RCLCPP_INFO(node->get_logger(), "Executing Joint-Space Pick and Place");

    // A) Pick Sequence
    if (!set_gripper(gripper_open)) return 1;
    if (!go_joint(approach_pick_joint)) return 1;
    if (!go_joint(pick_pose_joint)) return 1;
    if (!set_gripper(gripper_close)) return 1;

    // B) Place Sequence
    if (!go_joint(lift_pose_joint)) return 1;
    if (!go_joint(approach_place_joint)) return 1;
    if (!go_joint(place_pose_joint)) return 1;
    if (!set_gripper(gripper_open)) return 1;

    // C) Exit
    if (!go_joint(retreat_pose_joint)) return 1;

    RCLCPP_INFO(node->get_logger(), "test_xarm_pick_place over successfully");
    return 0;
}