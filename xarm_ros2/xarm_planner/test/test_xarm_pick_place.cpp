#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

// --- HELPER FUNCTION: Convert Roll, Pitch, Yaw (in degrees) to Quaternion ---
// Uses the verified sequence: Yaw -> Pitch -> Roll
geometry_msgs::msg::Quaternion createQuaternionFromRPY(double roll_deg, double pitch_deg, double yaw_deg)
{
    // Convert degrees to radians
    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0;

    tf2::Quaternion q_yaw, q_pitch, q_roll;
    q_yaw.setRPY(0, 0, yaw);         // Rotation about Z
    q_pitch.setRPY(0, pitch, 0);     // Rotation about Y
    q_roll.setRPY(roll, 0, 0);       // Rotation about X

    // Multiply in your verified order: Yaw -> Pitch -> Roll
    tf2::Quaternion q_total = q_yaw * q_pitch * q_roll;

    geometry_msgs::msg::Quaternion msg_quat;
    msg_quat.x = q_total.x();
    msg_quat.y = q_total.y();
    msg_quat.z = q_total.z();
    msg_quat.w = q_total.w();
    return msg_quat;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("mtc_safe_picker", node_options);

    xarm_planner::XArmPlanner arm_planner(node, "xarm7"); 
    xarm_planner::XArmPlanner gripper_planner(node, "xarm_gripper");
    moveit::planning_interface::PlanningSceneInterface psi;

    // --- DEFINE YOUR ORIENTATION HERE IN DEGREES ---
    // Change these values easily whenever your TCP orientation changes!
    double target_roll = -180.0;
    double target_pitch = 0.0;
    double target_yaw = -90.0;

    geometry_msgs::msg::Quaternion target_orientation = createQuaternionFromRPY(target_roll, target_pitch, target_yaw);

    // 1. Setup the "Target Cube" (Red)
    auto const target_cube = [] {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "world";
        obj.id = "target_cube";
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {0.06, 0.06, 0.06};
        geometry_msgs::msg::Pose pose;
        pose.orientation.x = 1.0; pose.orientation.w = 0.0; 
        pose.position.x = -0.1051; pose.position.y = 0.4166; pose.position.z = 0.03; 
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
    std::vector<double> gripper_close(6, 0.31);

    // --- HELPERS ---
    auto run_pose = [&](const geometry_msgs::msg::Pose &pose, const char *name) -> bool {
        if (!arm_planner.planPoseTarget(pose)) {
            RCLCPP_ERROR(node->get_logger(), "%s: planning failed", name);
            return false;
        }
        if (!arm_planner.executePath()) {
            RCLCPP_ERROR(node->get_logger(), "%s: execution failed", name);
            return false;
        }
        return true;
    };

    auto run_cartesian = [&](const geometry_msgs::msg::Pose &pose, const char *name) -> bool {
        std::vector<geometry_msgs::msg::Pose> waypoints{pose};
        if (!arm_planner.planCartesianPath(waypoints)) {
            RCLCPP_ERROR(node->get_logger(), "%s: Cartesian planning failed", name);
            return false;
        }
        if (!arm_planner.executePath()) {
            RCLCPP_ERROR(node->get_logger(), "%s: execution failed", name);
            return false;
        }
        return true;
    };

    auto run_gripper = [&](const std::vector<double> &target, const char *name) -> bool {
        if (!gripper_planner.planJointTarget(target)) {
            RCLCPP_ERROR(node->get_logger(), "%s: gripper planning failed", name);
            return false;
        }
        if (!gripper_planner.executePath()) {
            RCLCPP_ERROR(node->get_logger(), "%s: gripper execution failed", name);
            return false;
        }
        return true;
    };

    // Open Gripper initially
    RCLCPP_INFO(node->get_logger(), "STAGE: Opening Gripper (initial)");
    if (!run_gripper(gripper_open, "Initial Gripper Open")) return 1;

    // 4. STAGE: Approach (above pick)
    geometry_msgs::msg::Pose approach_pose;
    approach_pose.orientation = target_orientation;
    approach_pose.position.x = -0.1051; approach_pose.position.y = 0.4166; approach_pose.position.z = 0.20;
    
    RCLCPP_INFO(node->get_logger(), "Executing STAGE: Approach");
    if (!run_pose(approach_pose, "Approach")) return 1;

    // STAGE: Cartesian Lowering
    geometry_msgs::msg::Pose pick_pose;
    pick_pose.orientation = target_orientation;
    pick_pose.position.x = -0.1051; pick_pose.position.y = 0.4166; pick_pose.position.z = 0.020;
    
    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Lowering");
    if (!run_cartesian(pick_pose, "Cartesian Lowering")) return 1;

    // Attach collision object
    moveit_msgs::msg::AttachedCollisionObject allow_touch;
    allow_touch.link_name = "link_tcp"; 
    allow_touch.object = target_cube;
    allow_touch.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    allow_touch.touch_links = {"left_finger", "right_finger", "left_inner_knuckle", "right_inner_knuckle", "link_tcp"};
    psi.applyAttachedCollisionObject(allow_touch);

    // STAGE: Grasp
    RCLCPP_INFO(node->get_logger(), "STAGE: Grasp - Sending close command");
    if (!run_gripper(gripper_close, "Grasp")) return 1;

    // 5. STAGE: Cartesian Lift
    geometry_msgs::msg::Pose above_pick_pose;
    above_pick_pose.orientation = target_orientation;
    above_pick_pose.position.x = -0.1051; above_pick_pose.position.y = 0.4166; above_pick_pose.position.z = 0.20;
    
    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Lift");
    if (!run_cartesian(above_pick_pose, "Cartesian Lift")) return 1;

    // 6. STAGE: Above place position
    geometry_msgs::msg::Pose above_place_pose;
    above_place_pose.orientation = target_orientation;
    above_place_pose.position.x = -0.3503; above_place_pose.position.y = 0.58; above_place_pose.position.z = 0.20;

    RCLCPP_INFO(node->get_logger(), "STAGE: Move to above place pose (free-space)");
    if (!run_pose(above_place_pose, "Above Place")) return 1;

    // 7. STAGE: Place position
    geometry_msgs::msg::Pose place_pose;
    place_pose.orientation = target_orientation;
    place_pose.position.x = -0.3503; place_pose.position.y = 0.58; place_pose.position.z = 0.020;
    
    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Place");
    if (!run_cartesian(place_pose, "Cartesian Place")) return 1;

    // STAGE: Opening of gripper
    RCLCPP_INFO(node->get_logger(), "STAGE: Opening Gripper");
    if (!run_gripper(gripper_open, "Open Gripper")) return 1;

    // Final Stage: Clear pose
    geometry_msgs::msg::Pose clear_pose;
    clear_pose.orientation = target_orientation;
    clear_pose.position.x = -0.3503; clear_pose.position.y = 0.58; clear_pose.position.z = 0.20;
    
    RCLCPP_INFO(node->get_logger(), "STAGE: Final clearing move");
    if (!run_cartesian(clear_pose, "Final Clearing Move")) return 1;

    RCLCPP_INFO(node->get_logger(), "Pick and Place Sequence Complete!");
    rclcpp::shutdown();
    return 0;
}