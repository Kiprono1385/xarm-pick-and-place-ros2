#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

// --- SERVICE HEADER ---
#include "linkattacher_msgs/srv/attach_link.hpp"
#include "linkattacher_msgs/srv/detach_link.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("mtc_safe_picker", node_options);

    xarm_planner::XArmPlanner arm_planner(node, "xarm7"); 
    xarm_planner::XArmPlanner gripper_planner(node, "xarm_gripper");
    moveit::planning_interface::PlanningSceneInterface psi;

    // --- SERVICE CLIENTS (FIXED LINE 26) ---
    auto attach_client = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    auto detach_client = node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

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

    // 3. Apply Colors
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

    std::vector<double> gripper_open(6, 0.0);
    std::vector<double> gripper_close(6, 0.42);

    // 4. STAGE: Approach
    geometry_msgs::msg::Pose approach_pose;
    approach_pose.orientation.x = 1.0; approach_pose.orientation.w = 0.0; 
    approach_pose.position.x = -0.34; approach_pose.position.y = -0.20; approach_pose.position.z = 0.15;
    
    RCLCPP_INFO(node->get_logger(), "Executing STAGE: Approach");
    if (!arm_planner.planPoseTarget(approach_pose)) return 1;
    arm_planner.executePath();

    // STAGE: Cartesian Lowering
    geometry_msgs::msg::Pose pick_pose;
    pick_pose.orientation.x = 1.0; pick_pose.orientation.w = 0.0; 
    pick_pose.position.x = -0.34; pick_pose.position.y = -0.20; pick_pose.position.z = 0.015;
    
    std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;
    cartesian_waypoints.push_back(pick_pose);

    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Lowering");
    if (!arm_planner.planCartesianPath(cartesian_waypoints)) return 1;
    arm_planner.executePath();

    // --- STAGE: Enabling 'Ghost' mode ---
    moveit_msgs::msg::AttachedCollisionObject allow_touch;
    allow_touch.link_name = "link_tcp"; 
    allow_touch.object = target_cube;
    allow_touch.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    allow_touch.touch_links = {"left_finger", "right_finger", "left_inner_knuckle", "right_inner_knuckle", "link_tcp"};
    psi.applyAttachedCollisionObject(allow_touch);
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    // STAGE: Grasp
    RCLCPP_INFO(node->get_logger(), "STAGE: Grasp - Sending close command");
    if (gripper_planner.planJointTarget(gripper_close)) {
        gripper_planner.executePath(); 
        rclcpp::sleep_for(std::chrono::milliseconds(2000)); 
        
        // --- ATTACH LINK IN GAZEBO ---
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "UF_ROBOT";
        request->link1_name = "link7";
        request->model2_name = "target_cube";
        request->link2_name = "link";

        if (!attach_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node->get_logger(), "Service /ATTACHLINK not available!");
        } else {
            auto result = attach_client->async_send_request(request);
            rclcpp::spin_until_future_complete(node, result);
        }
    }

    // 5. STAGE: Cartesian Lift
    geometry_msgs::msg::Pose above_pick_pose;
    above_pick_pose.orientation.x = 1.0; above_pick_pose.orientation.w = 0.0; 
    above_pick_pose.position.x = -0.34; above_pick_pose.position.y = -0.20; above_pick_pose.position.z = 0.15;
    
    std::vector<geometry_msgs::msg::Pose> lift_waypoints;
    lift_waypoints.push_back(above_pick_pose);

    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Lift");
    if (arm_planner.planCartesianPath(lift_waypoints)) {
        arm_planner.executePath();
    }

    // 6. STAGE: Above place position
    geometry_msgs::msg::Pose above_place_pose;
    above_place_pose.orientation.x = 1.0; above_place_pose.orientation.w = 0.0; 
    above_place_pose.position.x = -0.34; above_place_pose.position.y = -0.40; above_place_pose.position.z = 0.15;

    std::vector<geometry_msgs::msg::Pose> place_waypoints;
    place_waypoints.push_back(above_place_pose);

    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian sideways");
    if (arm_planner.planCartesianPath(place_waypoints)) {
        arm_planner.executePath();
    }

    // 7. STAGE: Place position
    geometry_msgs::msg::Pose place_pose;
    place_pose.orientation.x = 1.0; place_pose.orientation.w = 0.0; 
    place_pose.position.x = -0.34; place_pose.position.y = -0.40; place_pose.position.z = 0.020;
    
    std::vector<geometry_msgs::msg::Pose> final_drop_waypoints;
    final_drop_waypoints.push_back(place_pose);

    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Place");
    if (arm_planner.planCartesianPath(final_drop_waypoints)) {
        arm_planner.executePath();
    }

    // STAGE: Opening of gripper and Detaching
    RCLCPP_INFO(node->get_logger(), "STAGE: Opening Gripper and Detaching Cube");

    if (gripper_planner.planJointTarget(gripper_open)) {
        gripper_planner.executePath(); 
        rclcpp::sleep_for(std::chrono::milliseconds(2000)); 
        
        // 1. FIXED: Removed 'auto' re-declaration, using the client from line 26
        auto detach_request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        detach_request->model1_name = "UF_ROBOT";
        detach_request->link1_name = "link7";
        detach_request->model2_name = "target_cube";
        detach_request->link2_name = "link";

        if (!detach_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(node->get_logger(), "Service /DETACHLINK still not available!");
        } else {
            auto result = detach_client->async_send_request(detach_request);
            if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Gazebo: Cube successfully detached.");
                
                moveit_msgs::msg::AttachedCollisionObject detach_object;
                detach_object.object.id = "target_cube";
                detach_object.link_name = "link_tcp";
                detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
                psi.applyAttachedCollisionObject(detach_object);
            }
        }
    }

    // Final Stage: Move back to above place position to clear the object
    geometry_msgs::msg::Pose clear_pose;
    clear_pose.orientation.x = 1.0; clear_pose.orientation.w = 0.0; 
    clear_pose.position.x = -0.34; clear_pose.position.y = -0.40; clear_pose.position.z = 0.15;
    
    std::vector<geometry_msgs::msg::Pose> clear_waypoints;
    clear_waypoints.push_back(clear_pose);

    RCLCPP_INFO(node->get_logger(), "STAGE: Final clearing move");
    if (arm_planner.planCartesianPath(clear_waypoints)) {
        arm_planner.executePath();
    }

    RCLCPP_INFO(node->get_logger(), "Pick and Place Sequence Complete!");
    rclcpp::shutdown();
    return 0;
}