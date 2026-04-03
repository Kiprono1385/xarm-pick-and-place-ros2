/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 * Author: Vinman <vinman.cub@gmail.com>
 * Modified by Brian Kiprono
 * Changes: Fixed private member access error and scoped lambda variables
 */

#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("test_xarm_pick_place", node_options);
    
    signal(SIGINT, SIG_DFL);

    // Determine MoveIt group names
    int dof;
    node->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));

    std::string arm_group = prefix + robot_type + std::to_string(dof);
    std::string gripper_group = prefix + robot_type + "_gripper";

    // Initialize Planners
    xarm_planner::XArmPlanner arm_planner(node, arm_group);
    xarm_planner::XArmPlanner gripper_planner(node, gripper_group);

    // --- 1. SPAWN CUBE (Fixed Lambda Style) ---
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // We manually set the frame_id to "world" or "link_base" to avoid the private member error
    auto const collision_object = [] {
        moveit_msgs::msg::CollisionObject collision_object;
        // Check your RViz 'Fixed Frame'. If it's link_base, use that here.
        collision_object.header.frame_id = "link_base"; // or "world"
        collision_object.id = "target_cube";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.05; // 5cm
        primitive.dimensions[primitive.BOX_Y] = 0.05;
        primitive.dimensions[primitive.BOX_Z] = 0.05;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = -0.34; // Table center
        box_pose.position.y = -0.20; // Table center
        box_pose.position.z = 0.025; // 2.5cm up to sit on table

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    RCLCPP_INFO(node->get_logger(), "Applying collision object...");
    planning_scene_interface.applyCollisionObject(collision_object);

    // --- 2. JOINT TARGETS ---
    std::vector<double> gripper_open(6, 0.0);
    std::vector<double> gripper_close(6, 0.855);

    // Radians for 7 joints
    std::vector<double> approach_pick_joint  = {3.1067, 0.2269, 0.0000, 1.3090, 0.0000, 1.0297, 3.1067};
    std::vector<double> pick_pose_joint      = {3.1067, 0.4014, 0.0000, 1.1868, 0.0000, 0.7505, 3.1067};
    std::vector<double> lift_pose_joint      = {3.1067, 0.2269, 0.0000, 1.3090, 0.0000, 1.0297, 3.1067};  
    std::vector<double> approach_place_joint = {0.5411, -0.9076, -2.0944, 2.2515, -0.7679, 1.6581, -1.8675};
    std::vector<double> place_pose_joint     = {0.4014, -1.0647, -2.1817, 1.8675, -0.8901, 1.2217, -1.7279};
    std::vector<double> retreat_pose_joint   = {0.5411, -0.9076, -2.0944, 2.2515, -0.7679, 1.6581, -1.8675};

    auto go_joint = [&](const std::vector<double> &target) {
        if (!arm_planner.planJointTarget(target)) return false;
        return arm_planner.executePath();
    };

    auto set_gripper = [&](const std::vector<double> &target) {
        if (!gripper_planner.planJointTarget(target)) return false;
        return gripper_planner.executePath();
    };

    // --- 3. EXECUTION ---
    if (!set_gripper(gripper_open)) return 1;
    if (!go_joint(approach_pick_joint)) return 1;
    if (!go_joint(pick_pose_joint)) return 1;
    
    RCLCPP_INFO(node->get_logger(), "Closing gripper...");
    if (!set_gripper(gripper_close)) return 1;

    if (!go_joint(lift_pose_joint)) return 1;
    if (!go_joint(approach_place_joint)) return 1;
    if (!go_joint(place_pose_joint)) return 1;
    
    if (!set_gripper(gripper_open)) return 1;
    if (!go_joint(retreat_pose_joint)) return 1;

    RCLCPP_INFO(node->get_logger(), "Done.");
    return 0;
}