#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("mtc_safe_picker", node_options);

    xarm_planner::XArmPlanner arm_planner(node, "xarm7"); 
    xarm_planner::XArmPlanner gripper_planner(node, "xarm_gripper");
    moveit::planning_interface::PlanningSceneInterface psi;

    // 1. Setup the "Target Cube" (Red)
    auto const target_cube = [] {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "world";
        obj.id = "target_cube";
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {0.05, 0.05, 0.05};
        geometry_msgs::msg::Pose pose;
        pose.orientation.x = 1.0; pose.orientation.w = 0.0; 
        pose.position.x = -0.34; pose.position.y = -0.20; pose.position.z = 0.025; 
        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        return obj;
    }();

    // 2. Setup the "Table Surface" (Brown)
    auto const table_surface = [] {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "world";
        obj.id = "table_surface";
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {1.5, 0.8, 0.01}; 
        geometry_msgs::msg::Pose pose;
        pose.orientation.z = 0.7071; pose.orientation.w = 0.7071;
        pose.position.x = -0.34; pose.position.y = -0.20; pose.position.z = -0.005; 
        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        return obj;
    }();

    psi.applyCollisionObject(target_cube);
    psi.applyCollisionObject(table_surface);

    // 3. Apply Colors (RETAINED)
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    moveit_msgs::msg::ObjectColor cube_color;
    cube_color.id = "target_cube";
    cube_color.color.r = 1.0; cube_color.color.a = 1.0;
    moveit_msgs::msg::ObjectColor table_color;
    table_color.id = "table_surface";
    table_color.color.r = 0.58; table_color.color.g = 0.29; table_color.color.b = 0.0; table_color.color.a = 0.8;
    planning_scene.object_colors.push_back(cube_color);
    planning_scene.object_colors.push_back(table_color);
    psi.applyPlanningScene(planning_scene);

    // --- 2. DEFINE GRIPPER VECTORS (6-element format) ---
    std::vector<double> gripper_open(6, 0.0);
    std::vector<double> gripper_close(6, 0.46);

    // 4. STAGE: Approach (Safety Included)
    geometry_msgs::msg::Pose approach_pose;
    approach_pose.orientation.x = 1.0; approach_pose.orientation.w = 0.0; 
    approach_pose.position.x = -0.34; approach_pose.position.y = -0.20; approach_pose.position.z = 0.15;
    
    RCLCPP_INFO(node->get_logger(), "Executing STAGE: Approach");
    if (!arm_planner.planPoseTarget(approach_pose)) {
        RCLCPP_ERROR(node->get_logger(), "Approach planning failed! Stopping.");
        return 1;
    }
    arm_planner.executePath();


    
    // 5. STAGE: Cartesian Pick (THE STRAIGHT LINE MOVE)
    geometry_msgs::msg::Pose pick_pose;
    pick_pose.orientation.x = 1.0; pick_pose.orientation.w = 0.0; 
    pick_pose.position.x = -0.34; pick_pose.position.y = -0.20; pick_pose.position.z = 0.015;
    
    std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;
    cartesian_waypoints.push_back(pick_pose);

    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Lowering");
    // Using planCartesianPath instead of planPoseTarget
    if (!arm_planner.planCartesianPath(cartesian_waypoints)) {
        RCLCPP_ERROR(node->get_logger(), "Cartesian Pick Planning Failed!");
        return 1;
    }
    arm_planner.executePath();

    // --- STAGE: THE "GHOST" (Ignore collision before pick) ---
    RCLCPP_INFO(node->get_logger(), "STAGE: Enabling 'Ghost' mode for cube contact");
    moveit_msgs::msg::AttachedCollisionObject allow_touch;
    allow_touch.link_name = "link_tcp"; 
    allow_touch.object = target_cube;
    allow_touch.object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Add every part of the gripper that might touch the cube
    allow_touch.touch_links = {
        "left_finger", "right_finger", 
        "left_inner_knuckle", "right_inner_knuckle", 
        "left_outer_knuckle", "right_outer_knuckle",
        "link_tcp", "xarm_gripper_base_link"
    };
    psi.applyAttachedCollisionObject(allow_touch);
    // Short delay to let the Planning Scene "Referee" update the rules
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    

    // 7. STAGE: Grasp (Using the working format)
    // 1. Print to terminal (Optional, for your logs)
    RCLCPP_INFO(node->get_logger(), "STAGE: Grasp - Sending close command");

    // 2. PLAN: Calculate the move using your stored 'gripper_close' vector (6, 0.85)
    if (gripper_planner.planJointTarget(gripper_close)) {
        
        // 3. EXECUTE: This is the command that actually MOVES the gripper
        gripper_planner.executePath(); 
        
        // 4. WAIT: Give Gazebo 2 seconds to finish the physical movement
        rclcpp::sleep_for(std::chrono::milliseconds(2000)); 
        
        RCLCPP_INFO(node->get_logger(), "Gripper closed successfully.");
    } else {
        // This only runs if the math failed (e.g., joint limits exceeded)
        RCLCPP_ERROR(node->get_logger(), "Gripper planning failed! Motor will not move.");
        return 1;
    }

    // 5. STAGE: Cartesian After Pick (THE STRAIGHT LINE MOVE)
    geometry_msgs::msg::Pose above_pick_pose;
    above_pick_pose.orientation.x = 1.0; above_pick_pose.orientation.w = 0.0; 
    above_pick_pose.position.x = -0.34; above_pick_pose.position.y = -0.20; above_pick_pose.position.z = 0.15;
    
   
    cartesian_waypoints.push_back(above_pick_pose);

    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Above Pick");
    // Using planCartesianPath instead of planPoseTarget
    if (!arm_planner.planCartesianPath(cartesian_waypoints)) {
        RCLCPP_ERROR(node->get_logger(), "Cartesian After Pick Planning Failed!");
        return 1;
    }
    
    arm_planner.executePath();

    RCLCPP_INFO(node->get_logger(), "MTC-style Safe Pick Sequence Complete!");
    rclcpp::shutdown();
    return 0;
}