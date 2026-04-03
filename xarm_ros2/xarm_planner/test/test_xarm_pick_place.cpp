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
    auto node = rclcpp::Node::make_shared("mtc_final_picker", node_options);

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

    auto const table_surface = [] {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "world";
        obj.id = "table_surface";
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        
        // Standard dimensions (Length, Width, Thickness)
        primitive.dimensions = {1.5, 0.8, 0.01}; 
        
        geometry_msgs::msg::Pose pose;
        
        // --- APPLY 90 DEGREE ROTATION ON Y-AXIS ---
        // Formula: w = cos(45°), y = sin(45°)
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.7071; // sin(45°)
        pose.orientation.w = 0.7071;

        // Position coordinates from your provided snippet
        pose.position.x = -0.34; 
        pose.position.y = -0.20; 
        pose.position.z = -0.005; 

        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        return obj;
    }();

    psi.applyCollisionObject(target_cube);
    psi.applyCollisionObject(table_surface);

    // 3. Apply Colors via PlanningScene Diff
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

    // 4. STAGE: Approach
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 1.0; target_pose.orientation.w = 0.0; 
    target_pose.position.x = -0.34; target_pose.position.y = -0.20; target_pose.position.z = 0.15;

    RCLCPP_INFO(node->get_logger(), "Executing STAGE: Approach");
    if (arm_planner.planPoseTarget(target_pose)) arm_planner.executePath();

    // 5. STAGE: Pre-Attach (Collision Bypass)
    moveit_msgs::msg::AttachedCollisionObject allow_touch;
    allow_touch.link_name = "link_tcp"; 
    allow_touch.object = target_cube;
    allow_touch.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyAttachedCollisionObject(allow_touch);

    // 6. STAGE: Lower to Grasp Height
    target_pose.position.z = 0.035; 
    if (arm_planner.planPoseTarget(target_pose)) arm_planner.executePath();

    // 7. STAGE: Grasp
    std::vector<double> gripper_close = {0.85}; 
    if (gripper_planner.planJointTarget(gripper_close)) {
        gripper_planner.executePath();
        rclcpp::sleep_for(std::chrono::milliseconds(2000)); 
    }

    // 8. STAGE: Lift
    target_pose.position.z = 0.25; 
    if (arm_planner.planPoseTarget(target_pose)) {
        arm_planner.executePath();
        RCLCPP_INFO(node->get_logger(), "MTC-style Pick Sequence Complete!");
    }

    rclcpp::shutdown();
    return 0;
}