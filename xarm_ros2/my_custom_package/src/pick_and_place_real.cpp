#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
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
#include <cmath>
#include <limits>
#include <string>

// One step of the pre-planned sequence
struct Step {
    enum Type { ARM, GRIPPER_OPEN, GRIPPER_CLOSE, HANDSHAKE } type;
    std::string name;
    moveit_msgs::msg::RobotTrajectory traj;   // only used for ARM steps
};

class PickAndPlaceRobot {
public:
    PickAndPlaceRobot(rclcpp::Node::SharedPtr node)
        : node_(node),
          arm_move_group_(node_, "xarm7"),
          gripper_move_group_(node_, "xarm_gripper")
    {
        gripper_open_ = {0.42, 0.42, 0.42, 0.42, 0.42, 0.42};
        gripper_close_ = {0.49, 0.49, 0.49, 0.49, 0.49, 0.49};

        place_pose_.orientation.x = 1.0;
        place_pose_.orientation.w = 0.0;
        place_pose_.position.x = 0.5837;
        place_pose_.position.y = -0.331;
        place_pose_.position.z = 0.130;
        above_offset_ = 0.100;        // above place
        pick_above_offset_ = 0.140;   // above pick

        arm_move_group_.setPlanningTime(5.0);
        arm_move_group_.setNumPlanningAttempts(10);
        arm_move_group_.setGoalJointTolerance(1e-3);

        // Optional: speed of the real motions. Check what your config uses before raising it.
        // arm_move_group_.setMaxVelocityScalingFactor(0.3);
        // arm_move_group_.setMaxAccelerationScalingFactor(0.3);
    }

    moveit::core::RobotStatePtr currentState() { return arm_move_group_.getCurrentState(); }

    // ------------------------------------------------------------------
    // Environment
    // ------------------------------------------------------------------
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

        std::string station1_path = package_share_dir + "/object_scenes/Assembly_Distribution_Station_MCDFinal.stl";
        geometry_msgs::msg::Pose pose1;
        pose1.position.x = 0.35; pose1.position.y = -0.208; pose1.position.z = -0.36;
        pose1.orientation.w = 1.0;
        auto distribution_station = createMeshCollisionObject("assembly_distribution_station", station1_path, pose1);

        std::string station2_path = package_share_dir + "/object_scenes/Assembly_Sorting_Station_MCDFinal.stl";
        geometry_msgs::msg::Pose pose2;
        pose2.position.x = 0.35; pose2.position.y = 0.29; pose2.position.z = -0.37;
        pose2.orientation.z = 0.7071; pose2.orientation.w = 0.7071;
        auto sorting_station = createMeshCollisionObject("assembly_sorting_station", station2_path, pose2);

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
        psi_.applyCollisionObject(makeBoxObstacle("obst2", 0.0653, 0.0778, 0.1306, 0.40695, -0.0755, 0.0653));
        psi_.applyCollisionObject(makeBoxObstacle("obst3", 0.2129, 0.1553, 0.1813, 0.41105, -0.28275, 0.09065));
        psi_.applyCollisionObject(makeBoxObstacle("obst4", 0.01, 2.0, 1.159, -0.2962, 0, 0.5795));
        psi_.applyCollisionObject(makeBoxObstacle("obst5", 2.0, 2.0, 0.01, 0, 0, 0.7408));

        RCLCPP_INFO(node_->get_logger(), "Environment collision objects loaded successfully.");
    }

    // ------------------------------------------------------------------
    // Gripper
    // ------------------------------------------------------------------
    bool openGripper() {
        RCLCPP_INFO(node_->get_logger(), "STAGE: Opening Gripper");
        gripper_move_group_.setJointValueTarget(gripper_open_);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (gripper_move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return gripper_move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        }
        return false;
    }

    bool closeGripper() {
        RCLCPP_INFO(node_->get_logger(), "STAGE: Closing Gripper");
        gripper_move_group_.setJointValueTarget(gripper_close_);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (gripper_move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return gripper_move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        }
        return false;
    }

    // ------------------------------------------------------------------
    // PHASE 1: approach configurations (faster version)
    //  - short IK timeout
    //  - coarser Cartesian check step
    //  - stops after `max_valid` good candidates instead of trying all seeds
    // ------------------------------------------------------------------
    bool findApproachJoints(const geometry_msgs::msg::Pose& target,
                            double z_offset,
                            const std::vector<double>& seed,
                            std::vector<double>& above_joints,
                            int max_tries = 30,
                            int max_valid = 3)
    {
        const auto* jmg = arm_move_group_.getRobotModel()->getJointModelGroup("xarm7");
        const std::string ee_link = arm_move_group_.getEndEffectorLink();

        geometry_msgs::msg::Pose above = target;
        above.position.z += z_offset;

        std::vector<double> ref = seed;
        auto current = arm_move_group_.getCurrentState();
        if (ref.empty()) current->copyJointGroupPositions(jmg, ref);

        double best_dist = std::numeric_limits<double>::max();
        int valid = 0;
        bool found = false;

        for (int i = 0; i < max_tries && valid < max_valid; ++i) {
            moveit::core::RobotState state(*current);

            if (i == 0) state.setJointGroupPositions(jmg, ref);
            else        state.setToRandomPositions(jmg);

            if (!state.setFromIK(jmg, target, ee_link, 0.02)) continue;

            arm_move_group_.setStartState(state);
            moveit_msgs::msg::RobotTrajectory traj;
            double fraction = arm_move_group_.computeCartesianPath({above}, 0.01, 0.0, traj, true);

            if (fraction < 0.99 || traj.joint_trajectory.points.empty()) continue;

            ++valid;
            const auto& q = traj.joint_trajectory.points.back().positions;

            double dist = 0.0;
            for (size_t j = 0; j < q.size() && j < ref.size(); ++j)
                dist += (q[j] - ref[j]) * (q[j] - ref[j]);

            if (dist < best_dist) {
                best_dist = dist;
                above_joints = q;
                found = true;
            }
        }

        arm_move_group_.setStartStateToCurrentState();   // always restore
        return found;
    }

    bool prepareApproaches(const std::vector<geometry_msgs::msg::Pose>& pick_positions) {
        above_pick_joints_.assign(pick_positions.size(), {});

        for (size_t i = 0; i < pick_positions.size(); ++i) {
            if (!findApproachJoints(pick_positions[i], pick_above_offset_, {}, above_pick_joints_[i])) {
                RCLCPP_ERROR(node_->get_logger(), "No valid approach configuration for pick %zu", i + 1);
                return false;
            }
            logJoints("above_pick " + std::to_string(i + 1), above_pick_joints_[i]);
        }

        if (!findApproachJoints(place_pose_, above_offset_, {}, above_place_joints_)) {
            RCLCPP_ERROR(node_->get_logger(), "No valid approach configuration for place pose");
            return false;
        }
        logJoints("above_place", above_place_joints_);

        RCLCPP_INFO(node_->get_logger(), "All approach configurations computed.");
        return true;
    }

    // ------------------------------------------------------------------
    // PHASE 2: plan the WHOLE sequence (nothing moves yet)
    // ------------------------------------------------------------------
    bool planAllCycles(const std::vector<geometry_msgs::msg::Pose>& pick_positions,
                       std::vector<Step>& steps)
    {
        moveit::core::RobotState state(*arm_move_group_.getCurrentState());

        for (size_t i = 0; i < pick_positions.size(); ++i) {
            RCLCPP_INFO(node_->get_logger(), "Planning cycle %zu of %zu...", i + 1, pick_positions.size());
            if (!planCycle(i, pick_positions[i], state, steps)) {
                RCLCPP_ERROR(node_->get_logger(), "Planning failed in cycle %zu. Nothing was executed.", i + 1);
                arm_move_group_.setStartStateToCurrentState();
                return false;
            }
        }
        arm_move_group_.setStartStateToCurrentState();
        RCLCPP_INFO(node_->get_logger(), "All cycles planned (%zu steps).", steps.size());
        return true;
    }

    // ------------------------------------------------------------------
    // PHASE 3: execute stored steps
    // ------------------------------------------------------------------
    bool executeSteps(const std::vector<Step>& steps) {
        arm_move_group_.setStartStateToCurrentState();

        for (size_t k = 0; k < steps.size(); ++k) {
            const Step& s = steps[k];
            RCLCPP_INFO(node_->get_logger(), "EXEC [%zu/%zu]: %s", k + 1, steps.size(), s.name.c_str());

            switch (s.type) {
                case Step::ARM: {
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = s.traj;
                    auto result = arm_move_group_.execute(plan);
                    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_ERROR(node_->get_logger(), "Execution of '%s' failed (error code %d). Stopping.",
                                     s.name.c_str(), result.val);
                        return false;
                    }
                    break;
                }
                case Step::GRIPPER_CLOSE:
                    if (!closeGripper()) { RCLCPP_ERROR(node_->get_logger(), "Close gripper failed."); return false; }
                    break;
                case Step::GRIPPER_OPEN:
                    if (!openGripper()) { RCLCPP_ERROR(node_->get_logger(), "Open gripper failed."); return false; }
                    break;
                case Step::HANDSHAKE:
                    sendOpcUaHandshake();
                    break;
            }
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

        UA_NodeId node_id = UA_NODEID_STRING(3, (char*)"\"ROBOT_COMMUNICATION\".\"robot_done\"");

        UA_Variant v;
        UA_Variant_init(&v);

        bool val_true = true;
        UA_Variant_setScalar(&v, &val_true, &UA_TYPES[UA_TYPES_BOOLEAN]);
        UA_StatusCode write_rc1 = UA_Client_writeValueAttribute(client, node_id, &v);
        if (write_rc1 != UA_STATUSCODE_GOOD) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to write True to PLC: %s", UA_StatusCode_name(write_rc1));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        bool val_false = false;
        UA_Variant_setScalar(&v, &val_false, &UA_TYPES[UA_TYPES_BOOLEAN]);
        UA_StatusCode write_rc2 = UA_Client_writeValueAttribute(client, node_id, &v);
        if (write_rc2 != UA_STATUSCODE_GOOD) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to write False to PLC: %s", UA_StatusCode_name(write_rc2));
        }

        UA_Client_disconnect(client);
        UA_Client_delete(client);
        RCLCPP_INFO(node_->get_logger(), "OPC UA handshake sequence completed.");
        return (write_rc1 == UA_STATUSCODE_GOOD && write_rc2 == UA_STATUSCODE_GOOD);
    }

private:
    // End state of a trajectory, used as the start state of the next plan
    moveit::core::RobotState endState(const moveit_msgs::msg::RobotTrajectory& traj,
                                      const moveit::core::RobotState& start)
    {
        moveit::core::RobotState s(start);
        const auto& jt = traj.joint_trajectory;
        s.setVariablePositions(jt.joint_names, jt.points.back().positions);
        s.update();
        return s;
    }

    bool planJoints(const std::string& name, const std::vector<double>& goal,
                    moveit::core::RobotState& state, std::vector<Step>& steps, int max_retries = 5)
    {
        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            arm_move_group_.setStartState(state);
            arm_move_group_.setJointValueTarget(goal);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (arm_move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
                !plan.trajectory_.joint_trajectory.points.empty())
            {
                state = endState(plan.trajectory_, state);
                steps.push_back({Step::ARM, name, plan.trajectory_});
                return true;
            }
            RCLCPP_WARN(node_->get_logger(), "[%s] planning attempt %d/%d failed", name.c_str(), attempt, max_retries);
        }
        RCLCPP_ERROR(node_->get_logger(), "[%s] planning failed", name.c_str());
        return false;
    }

    bool planCartesian(const std::string& name, const geometry_msgs::msg::Pose& target,
                       moveit::core::RobotState& state, std::vector<Step>& steps,
                       double min_fraction = 0.95)
    {
        arm_move_group_.setStartState(state);
        moveit_msgs::msg::RobotTrajectory traj;
        double fraction = arm_move_group_.computeCartesianPath({target}, 0.005, 0.0, traj, true);
        RCLCPP_INFO(node_->get_logger(), "[%s] Cartesian fraction %.2f", name.c_str(), fraction);
        if (fraction < min_fraction || traj.joint_trajectory.points.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Cartesian planning failed", name.c_str());
            return false;
        }
        state = endState(traj, state);
        steps.push_back({Step::ARM, name, traj});
        return true;
    }

    bool planCycle(size_t i, const geometry_msgs::msg::Pose& pick,
                   moveit::core::RobotState& state, std::vector<Step>& steps)
    {
        geometry_msgs::msg::Pose above_pick = pick;
        above_pick.position.z += pick_above_offset_;
        geometry_msgs::msg::Pose above_place = place_pose_;
        above_place.position.z += above_offset_;

        const std::string c = "c" + std::to_string(i + 1) + " ";

        if (!planJoints(c + "above_pick", above_pick_joints_[i], state, steps)) return false;
        if (!planCartesian(c + "descend_pick", pick, state, steps)) return false;
        steps.push_back({Step::GRIPPER_CLOSE, c + "close_gripper", moveit_msgs::msg::RobotTrajectory()});
        if (!planCartesian(c + "lift_pick", above_pick, state, steps)) return false;
        if (!planJoints(c + "above_place", above_place_joints_, state, steps)) return false;
        if (!planCartesian(c + "descend_place", place_pose_, state, steps)) return false;
        steps.push_back({Step::GRIPPER_OPEN,  c + "open_gripper",  moveit_msgs::msg::RobotTrajectory()});
        if (!planCartesian(c + "clear_place", above_place, state, steps, 0.8)) return false;
        steps.push_back({Step::HANDSHAKE,     c + "opcua_handshake", moveit_msgs::msg::RobotTrajectory()});
        return true;
    }

    void logJoints(const std::string& name, const std::vector<double>& q) {
        std::string s;
        for (double v : q) s += std::to_string(v * 180.0 / M_PI).substr(0, 7) + " ";
        RCLCPP_INFO(node_->get_logger(), "%s (deg): %s", name.c_str(), s.c_str());
    }

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface arm_move_group_;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_;
    moveit::planning_interface::PlanningSceneInterface psi_;

    std::vector<double> gripper_open_;
    std::vector<double> gripper_close_;
    std::vector<std::vector<double>> above_pick_joints_;
    std::vector<double> above_place_joints_;
    geometry_msgs::msg::Pose place_pose_;
    double above_offset_;
    double pick_above_offset_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("pick_and_place_oop_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    auto finish = [&](int code) {
        rclcpp::shutdown();
        spinner.join();
        return code;
    };

    PickAndPlaceRobot robot(node);
    robot.setupEnvironment();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::vector<geometry_msgs::msg::Pose> pick_positions(3);

    pick_positions[0].orientation.x = 1.0; pick_positions[0].orientation.w = 0.0;
    pick_positions[0].position.x = -0.1039; pick_positions[0].position.y = 0.416; pick_positions[0].position.z = 0.004;

    pick_positions[1].orientation.x = 1.0; pick_positions[1].orientation.w = 0.0;
    pick_positions[1].position.x = 0.0579; pick_positions[1].position.y = 0.4212; pick_positions[1].position.z = 0.004;

    pick_positions[2].orientation.x = 1.0; pick_positions[2].orientation.w = 0.0;
    pick_positions[2].position.x = 0.0566; pick_positions[2].position.y = 0.599; pick_positions[2].position.z = 0.004;

    // PHASE 1: above-pick / above-place joint configurations
    if (!robot.prepareApproaches(pick_positions)) {
        RCLCPP_ERROR(node->get_logger(), "No valid approach configurations. Aborting before any motion.");
        return finish(1);
    }

    // PHASE 2: plan everything (no motion)
    std::vector<Step> steps;
    if (!robot.planAllCycles(pick_positions, steps)) {
        return finish(1);
    }

    // PHASE 3: everything is planned OK -> go
    robot.openGripper();
    if (!robot.executeSteps(steps)) {
        RCLCPP_ERROR(node->get_logger(), "Execution aborted.");
        return finish(1);
    }

    RCLCPP_INFO(node->get_logger(), "All 3 pick and place cycles completed successfully!");
    return finish(0);
}