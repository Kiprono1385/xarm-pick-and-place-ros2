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

// --- OPEN62541 OPC UA HEADERS ---
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>

#include <thread>
#include <vector>
#include <chrono>

class PickAndPlaceRobot {
public:
    PickAndPlaceRobot(rclcpp::Node::SharedPtr node)
        : node_(node),
          arm_move_group_(node_, "xarm7"),
          gripper_move_group_(node_, "xarm_gripper")
    {
        // Define gripper joint targets
        gripper_open_ = {0.42, 0.42, 0.42, 0.42, 0.42, 0.42};
        gripper_close_ = {0.49, 0.49, 0.49, 0.49, 0.49, 0.49};

        // Define shared place location
        place_pose_.orientation.x = 1.0; 
        place_pose_.orientation.w = 0.0; 
        place_pose_.position.x = 0.5817;
        place_pose_.position.y = -0.331;
        place_pose_.position.z = 0.130;
        above_offset_ = 0.100;

        // Configure robust planning parameters
        arm_move_group_.setPlanningTime(5.0);       // Allow up to 5 seconds per plan
        arm_move_group_.setNumPlanningAttempts(10); // Try 10 internal randomized seeds per plan call
    }

    // Helper function to load an STL file, scale it, and fix it to the world frame
    moveit_msgs::msg::CollisionObject createMeshCollisionObject(
        const std::string& id,
        const std::string& absolute_path,
        const geometry_msgs::msg::Pose& pose,
        double scale = 0.001,
        const std::string& frame_id = "world")
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = id;

        Eigen::Vector3d scale_vector(scale, scale, scale);
        shapes::Mesh* m = shapes::createMeshFromResource("file://" + absolute_path, scale_vector);
        if (!m) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load mesh from: %s", absolute_path.c_str());
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

    void setupEnvironment() {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("my_custom_package");

        // 1. Assembly Distribution Station
        std::string station1_path = package_share_dir + "/object_scenes/Assembly_Distribution_Station_MCDFinal.stl";
        geometry_msgs::msg::Pose pose1;
        pose1.position.x = 0.35; pose1.position.y = -0.208; pose1.position.z = -0.36;
        pose1.orientation.w = 1.0;
        auto distribution_station = createMeshCollisionObject("assembly_distribution_station", station1_path, pose1);

        // 2. Assembly Sorting Station
        std::string station2_path = package_share_dir + "/object_scenes/Assembly_Sorting_Station_MCDFinal.stl";
        geometry_msgs::msg::Pose pose2;
        pose2.position.x = 0.35; pose2.position.y = 0.29; pose2.position.z = -0.37;
        pose2.orientation.z = 0.7071; pose2.orientation.w = 0.7071;
        auto sorting_station = createMeshCollisionObject("assembly_sorting_station", station2_path, pose2);

        // 3. Table Surface
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
            obj.operation = obj.ADD;
            return obj;
        }();

        // Environment Obstacles (HMI & Obstacles 1-5)
        auto makeBoxObstacle = [](const std::string& id, double dx, double dy, double dz, double px, double py, double pz) {
            moveit_msgs::msg::CollisionObject obj;
            obj.header.frame_id = "world";
            obj.id = id;
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {dx, dy, dz};
            geometry_msgs::msg::Pose pose;
            pose.orientation.z = 1.0; pose.orientation.w = 0.0;
            pose.position.x = px; pose.position.y = py; pose.position.z = pz;
            obj.primitives.push_back(primitive);
            obj.primitive_poses.push_back(pose);
            obj.operation = obj.ADD;
            return obj;
        };

        psi_.applyCollisionObject(distribution_station);
        psi_.applyCollisionObject(sorting_station);
        psi_.applyCollisionObject(table_surface);
        psi_.applyCollisionObject(makeBoxObstacle("hmi", 0.1697, 0.3441, 0.3288, 0.85245, 0.07175, 0.1644));
        psi_.applyCollisionObject(makeBoxObstacle("obst1", 0.1672, 0.1349, 0.2574, 0.5809, -0.18825, 0.1287));
        psi_.applyCollisionObject(makeBoxObstacle("obst2", 0.0653, 0.0778, 0.1166, 0.40695, -0.0755, 0.0583));
        psi_.applyCollisionObject(makeBoxObstacle("obst3", 0.2129, 0.1553, 0.1813, 0.41105, -0.28275, 0.09065));
        psi_.applyCollisionObject(makeBoxObstacle("obst4", 0.01, 2.0, 1.159, -0.2962, 0, 0.5795));
        psi_.applyCollisionObject(makeBoxObstacle("obst5", 2.0, 2.0, 0.01, 0, 0, 0.7408));

        RCLCPP_INFO(node_->get_logger(), "Environment collision objects loaded successfully.");
    }

    bool openGripper() {
        RCLCPP_INFO(node_->get_logger(), "STAGE: Opening Gripper");
        gripper_move_group_.setJointValueTarget(gripper_open_);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (gripper_move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            gripper_move_group_.execute(plan);
            return true;
        }
        return false;
    }

    bool closeGripper() {
        RCLCPP_INFO(node_->get_logger(), "STAGE: Closing Gripper");
        gripper_move_group_.setJointValueTarget(gripper_close_);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (gripper_move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            gripper_move_group_.execute(plan);
            return true;
        }
        return false;
    }

    // Helper method with automatic retries for free-space motion planning
    bool planAndExecuteWithRetry(const std::string& stage_name, int max_retries = 5) {
        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto error_code = arm_move_group_.plan(plan);
            
            if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Plan successful on attempt %d. Executing...", stage_name.c_str(), attempt);
                auto exec_result = arm_move_group_.execute(plan);
                arm_move_group_.clearPoseTargets();
                
                if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    return true;
                }
                RCLCPP_WARN(node_->get_logger(), "[%s] Execution failed (error code %d), retrying...", stage_name.c_str(), exec_result.val);
            } else {
                RCLCPP_WARN(node_->get_logger(), "[%s] Planning rejected/failed (error code %d) on attempt %d/%d. Retrying with new seed...", 
                            stage_name.c_str(), error_code.val, attempt, max_retries);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        
        arm_move_group_.clearPoseTargets();
        RCLCPP_ERROR(node_->get_logger(), "[%s] Failed all %d planning/execution attempts.", stage_name.c_str(), max_retries);
        return false;
    }

    bool executePickAndPlace(const geometry_msgs::msg::Pose& pick_target) {
        // 1. Approach above pick
        geometry_msgs::msg::Pose above_pick_pose = pick_target;
        above_pick_pose.position.z += 0.140;

        RCLCPP_INFO(node_->get_logger(), "STAGE: Move to Above Pick Pose");
        arm_move_group_.setPoseTarget(above_pick_pose);
        if (!planAndExecuteWithRetry("Above Pick Pose")) return false;

        // 2. Cartesian Lowering to Pick
        std::vector<geometry_msgs::msg::Pose> cartesian_waypoints = {pick_target};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_move_group_.computeCartesianPath(cartesian_waypoints, 0.01, 0.0, trajectory);
        RCLCPP_INFO(node_->get_logger(), "STAGE: Cartesian Lowering (Path fraction: %.2f)", fraction);
        if (fraction <= 0.8) return false;

        moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
        cart_plan.trajectory_ = trajectory;
        arm_move_group_.execute(cart_plan);

        // 3. Grasp workpiece
        if (!closeGripper()) return false;

        // 4. Cartesian Lift
        std::vector<geometry_msgs::msg::Pose> lift_waypoints = {above_pick_pose};
        RCLCPP_INFO(node_->get_logger(), "STAGE: Cartesian Lift");
        fraction = arm_move_group_.computeCartesianPath(lift_waypoints, 0.01, 0.0, trajectory);
        if (fraction <= 0.8) return false;

        moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
        lift_plan.trajectory_ = trajectory;
        arm_move_group_.execute(lift_plan);

        // 5. Move to Above Place Pose (Direct with retry mechanism)
        geometry_msgs::msg::Pose above_place_pose;
        above_place_pose.orientation.x = 1.0; above_place_pose.orientation.w = 0.0;
        above_place_pose.position.x = place_pose_.position.x;
        above_place_pose.position.y = place_pose_.position.y;
        above_place_pose.position.z = place_pose_.position.z + above_offset_;

        RCLCPP_INFO(node_->get_logger(), "STAGE: Move to Above Place Pose");
        arm_move_group_.setPoseTarget(above_place_pose);
        if (!planAndExecuteWithRetry("Above Place Pose")) return false;

        // 6. Cartesian Place
        std::vector<geometry_msgs::msg::Pose> final_drop_waypoints = {place_pose_};
        RCLCPP_INFO(node_->get_logger(), "STAGE: Cartesian Place");
        fraction = arm_move_group_.computeCartesianPath(final_drop_waypoints, 0.01, 0.0, trajectory);
        if (fraction <= 0.8) return false;

        moveit::planning_interface::MoveGroupInterface::Plan drop_plan;
        drop_plan.trajectory_ = trajectory;
        arm_move_group_.execute(drop_plan);

        // 7. Open Gripper to release
        if (!openGripper()) return false;

        // 8. Final Clearing Move
        std::vector<geometry_msgs::msg::Pose> clear_waypoints = {above_place_pose};
        RCLCPP_INFO(node_->get_logger(), "STAGE: Final Clearing Move");
        fraction = arm_move_group_.computeCartesianPath(clear_waypoints, 0.01, 0.0, trajectory);
        if (fraction > 0.8) {
            moveit::planning_interface::MoveGroupInterface::Plan clear_plan;
            clear_plan.trajectory_ = trajectory;
            arm_move_group_.execute(clear_plan);
        }

        return true;
    }

    bool sendOpcUaHandshake() {
        RCLCPP_INFO(node_->get_logger(), "Connecting to OPC UA server...");
        UA_Client* client = UA_Client_new();
        UA_ClientConfig* cfg = UA_Client_getConfig(client);
        UA_ClientConfig_setDefault(cfg);
        cfg->timeout = 3000;

        UA_StatusCode ua_rc = UA_Client_connect(client, "opc.tcp://172.16.40.21:4840");
        if (ua_rc != UA_STATUSCODE_GOOD) {
            RCLCPP_ERROR(node_->get_logger(), "OPC UA Connect failed: %s", UA_StatusCode_name(ua_rc));
            UA_Client_delete(client);
            return false;
        }

        const char* node_str = "\"DISTRIBUTION DB\".\"robot_done\"";
        
        // Write True
        UA_Variant v;
        UA_Variant_init(&v);
        bool val_true = true;
        UA_Variant_setScalar(&v, &val_true, &UA_TYPES[UA_TYPES_BOOLEAN]);
        UA_Client_writeValueAttribute(client, UA_NODEID_STRING(3, const_cast<char*>(node_str)), &v);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Write False
        bool val_false = false;
        UA_Variant_setScalar(&v, &val_false, &UA_TYPES[UA_TYPES_BOOLEAN]);
        UA_Client_writeValueAttribute(client, UA_NODEID_STRING(3, const_cast<char*>(node_str)), &v);

        UA_Client_disconnect(client);
        UA_Client_delete(client);
        RCLCPP_INFO(node_->get_logger(), "OPC UA handshake sequence completed successfully.");
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface arm_move_group_;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_;
    moveit::planning_interface::PlanningSceneInterface psi_;

    std::vector<double> gripper_open_;
    std::vector<double> gripper_close_;
    geometry_msgs::msg::Pose place_pose_;
    double above_offset_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("pick_and_place_oop_node", node_options);

    PickAndPlaceRobot robot(node);
    robot.setupEnvironment();

    // Initial Gripper Opening
    robot.openGripper();

    // Define 3 Different Pick Positions
    std::vector<geometry_msgs::msg::Pose> pick_positions(3);

    // Pick Position 1
    pick_positions[0].orientation.x = 1.0; pick_positions[0].orientation.w = 0.0;
    pick_positions[0].position.x = -0.1039; pick_positions[0].position.y = 0.416; pick_positions[0].position.z = 0.004;

    // Pick Position 2
    pick_positions[1].orientation.x = 1.0; pick_positions[1].orientation.w = 0.0;
    pick_positions[1].position.x = 0.0579; pick_positions[1].position.y = 0.4212; pick_positions[1].position.z = 0.004;

    // Pick Position 3
    pick_positions[2].orientation.x = 1.0; pick_positions[2].orientation.w = 0.0;
    pick_positions[2].position.x = 0.0566; pick_positions[2].position.y = 0.599; pick_positions[2].position.z = 0.004;

    // Loop through all three picking stations sequentially
    for (size_t i = 0; i < pick_positions.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "=== STARTING PICK & PLACE CYCLE %zu OF 3 ===", i + 1);
        if (!robot.executePickAndPlace(pick_positions[i])) {
            RCLCPP_ERROR(node->get_logger(), "Pick and place cycle %zu failed. Aborting.", i + 1);
            rclcpp::shutdown();
            return 1;
        }

        // Trigger OPC UA handshake immediately after each successful place
        robot.sendOpcUaHandshake();
    }

    RCLCPP_INFO(node->get_logger(), "All 3 pick and place cycles completed successfully!");
    rclcpp::shutdown();
    return 0;
}