#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// --- GEOMETRIC SHAPES & EIGEN FOR STL LOADING ---
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Core>

#include <thread>
#include <vector>
#include <chrono>

// Helper function to load an STL file, scale it, and fix it to the world frame
moveit_msgs::msg::CollisionObject createMeshCollisionObject(
    const std::string& id,
    const std::string& absolute_path,
    const geometry_msgs::msg::Pose& pose,
    double scale = 0.001,
    const std::string& frame_id = "world")
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id; // Explicitly fixed to "world"
    collision_object.id = id;

    Eigen::Vector3d scale_vector(scale, scale, scale);
    shapes::Mesh* m = shapes::createMeshFromResource("file://" + absolute_path, scale_vector);
    if (!m) {
        RCLCPP_ERROR(rclcpp::get_logger("mesh_loader"), "Failed to load mesh from: %s", absolute_path.c_str());
        return collision_object;
    }

    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    delete m;
    return collision_object;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("pick_and_place_real", node_options);

    // Standard MoveIt 2 Interfaces
    moveit::planning_interface::MoveGroupInterface arm_move_group(node, "xarm7");
    moveit::planning_interface::MoveGroupInterface gripper_move_group(node, "xarm_gripper");
    moveit::planning_interface::PlanningSceneInterface psi;

    // --- RESOLVE PACKAGE SHARE DIRECTORY AND LOAD SCENE STLs ---
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("my_custom_package");

    // 1. Assembly Distribution Station (Fixed to World)
    std::string station1_path = package_share_dir + "/object_scenes/Assembly_Distribution_Station_MCDFinal.stl";
    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 0.35;
    pose1.position.y = -0.24;
    pose1.position.z = -0.36;
    pose1.orientation.x = 0.0;
    pose1.orientation.y = 0.0;
    pose1.orientation.z = 0.0;
    pose1.orientation.w = 1.0;
    auto distribution_station = createMeshCollisionObject("assembly_distribution_station", station1_path, pose1, 0.001, "world");

    // 2. Assembly Sorting Station (Fixed to World, with 1.57 rad yaw rotation)
    std::string station2_path = package_share_dir + "/object_scenes/Assembly_Sorting_Station_MCDFinal.stl";
    geometry_msgs::msg::Pose pose2;
    pose2.position.x = 0.35;
    pose2.position.y = 0.29;
    pose2.position.z = -0.37;
    pose2.orientation.x = 0.0;
    pose2.orientation.y = 0.0;
    pose2.orientation.z = 0.7071; // sin(1.57 / 2)
    pose2.orientation.w = 0.7071; // cos(1.57 / 2)
    auto sorting_station = createMeshCollisionObject("assembly_sorting_station", station2_path, pose2, 0.001, "world");

    // 3. Setup the Target Cylinder (4cm diameter, 2.5cm height)
    auto const target_cylinder = [] {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "world";
        obj.id = "target_cube";
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        primitive.dimensions = {0.025, 0.020}; // {height: 2.5cm, radius: 2.0cm -> 4cm diameter}
        geometry_msgs::msg::Pose pose;
        pose.orientation.x = 1.0; pose.orientation.w = 0.0; 
        pose.position.x = -0.1039; pose.position.y = 0.416; pose.position.z = 0.0125; // cylinder center: half of 0.025 m height
        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        return obj;
    }();

    // 4. Setup the Table Surface
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

    psi.applyCollisionObject(distribution_station);
    psi.applyCollisionObject(sorting_station);
    psi.applyCollisionObject(target_cylinder);
    psi.applyCollisionObject(table_surface);

    // --- GRIPPER VALUES (Open: 0.0 | Closed on Cylinder: 0.502) ---
    std::vector<double> gripper_open = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> gripper_close = {0.49, 0.49, 0.49, 0.49, 0.49, 0.49};

    // --- PLACE LOCATION ---
    const double place_x = 0.5771;
    const double place_y = -0.368;
    const double place_z = 0.120;
    const double above_offset = 0.100; // 10 cm offset: above place z = 0.220 m

    // 6. STAGE: Approach (10 cm above pick)
    geometry_msgs::msg::Pose approach_pose;
    approach_pose.orientation.x = 1.0; approach_pose.orientation.w = 0.0; 
    approach_pose.position.x = -0.1039; approach_pose.position.y = 0.416; approach_pose.position.z = 0.140;
    
    RCLCPP_INFO(node->get_logger(), "Executing STAGE: Approach");
    arm_move_group.setPoseTarget(approach_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    if (arm_move_group.plan(plan1) != moveit::core::MoveItErrorCode::SUCCESS) return 1;
    arm_move_group.execute(plan1);

    // STAGE: Cartesian Lowering
    geometry_msgs::msg::Pose pick_pose;
    pick_pose.orientation.x = 1.0; pick_pose.orientation.w = 0.0; 
    pick_pose.position.x = -0.1039; pick_pose.position.y = 0.416; pick_pose.position.z = 0.004;
    
    std::vector<geometry_msgs::msg::Pose> cartesian_waypoints = {pick_pose};
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm_move_group.computeCartesianPath(cartesian_waypoints, 0.01, 0.0, trajectory);
    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Lowering (Path fraction: %.2f)", fraction);
    if (fraction > 0.8) {
        moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
        cart_plan.trajectory_ = trajectory;
        arm_move_group.execute(cart_plan);
    } else {
        return 1;
    }

    // STAGE: Grasp (Closing gripper on cylinder)
    RCLCPP_INFO(node->get_logger(), "STAGE: Grasp - Closing gripper on cylinder");
    gripper_move_group.setJointValueTarget(gripper_close);
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    if (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        gripper_move_group.execute(gripper_plan); 
    }

    // 7. STAGE: Cartesian Lift
    geometry_msgs::msg::Pose above_pick_pose;
    above_pick_pose.orientation.x = 1.0; above_pick_pose.orientation.w = 0.0; 
    above_pick_pose.position.x = -0.1039; above_pick_pose.position.y = 0.416; above_pick_pose.position.z = 0.140;
    
    std::vector<geometry_msgs::msg::Pose> lift_waypoints = {above_pick_pose};
    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Lift");
    fraction = arm_move_group.computeCartesianPath(lift_waypoints, 0.01, 0.0, trajectory);
    if (fraction > 0.8) {
        moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
        lift_plan.trajectory_ = trajectory;
        arm_move_group.execute(lift_plan);
    }

    // 8. STAGE: Move to above place pose (free-space)
    geometry_msgs::msg::Pose above_place_pose;
    above_place_pose.orientation.x = 1.0; above_place_pose.orientation.w = 0.0; 
    above_place_pose.position.x = place_x;
    above_place_pose.position.y = place_y;
    above_place_pose.position.z = place_z + above_offset;

    RCLCPP_INFO(node->get_logger(), "STAGE: Move to above place pose (free-space)");
    arm_move_group.setPoseTarget(above_place_pose);
    moveit::planning_interface::MoveGroupInterface::Plan place_transit_plan;
    if (arm_move_group.plan(place_transit_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        arm_move_group.execute(place_transit_plan);
    } else {
        return 1;
    }

    // 9. STAGE: Cartesian Place
    geometry_msgs::msg::Pose place_pose;
    place_pose.orientation.x = 1.0; place_pose.orientation.w = 0.0; 
    place_pose.position.x = place_x;
    place_pose.position.y = place_y;
    place_pose.position.z = place_z;
    
    std::vector<geometry_msgs::msg::Pose> final_drop_waypoints = {place_pose};
    RCLCPP_INFO(node->get_logger(), "STAGE: Cartesian Place");
    fraction = arm_move_group.computeCartesianPath(final_drop_waypoints, 0.01, 0.0, trajectory);
    if (fraction > 0.8) {
        moveit::planning_interface::MoveGroupInterface::Plan drop_plan;
        drop_plan.trajectory_ = trajectory;
        arm_move_group.execute(drop_plan);
    }

    // STAGE: Opening Gripper
    RCLCPP_INFO(node->get_logger(), "STAGE: Opening Gripper");
    gripper_move_group.setJointValueTarget(gripper_open);
    if (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        gripper_move_group.execute(gripper_plan); 
    }

    // Final Stage: Clearing Move
    geometry_msgs::msg::Pose clear_pose;
    clear_pose.orientation.x = 1.0; clear_pose.orientation.w = 0.0; 
    clear_pose.position.x = place_x;
    clear_pose.position.y = place_y;
    clear_pose.position.z = place_z + above_offset;
    
    std::vector<geometry_msgs::msg::Pose> clear_waypoints = {clear_pose};
    RCLCPP_INFO(node->get_logger(), "STAGE: Final clearing move");
    fraction = arm_move_group.computeCartesianPath(clear_waypoints, 0.01, 0.0, trajectory);
    if (fraction > 0.8) {
        moveit::planning_interface::MoveGroupInterface::Plan clear_plan;
        clear_plan.trajectory_ = trajectory;
        arm_move_group.execute(clear_plan);
    }

    RCLCPP_INFO(node->get_logger(), "Real Pick and Place Sequence Complete!");
    rclcpp::shutdown();
    return 0;
}