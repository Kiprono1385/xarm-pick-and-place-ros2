# 🤖 UFactory xArm7 ROS 2 Pick & Place

This project extends the UFACTORY xArm7 ROS 2 framework with a custom pick-and-place implementation using MoveIt 2 motion planning.

---

## 🚀 Overview

A complete pick-and-place pipeline for the xArm7 robotic manipulator built using ROS 2 and MoveIt 2.
The implementation focuses on **reliable motion planning**, **safe joint limits**, and **modular execution**.

---

## ✨ Key Modifications

### 🧠 Pick-and-Place Logic

File: `test/test_xarm_pick_place.cpp`

* Uses **MoveGroup-based motion planning** (no Task Constructor)
* Carefully selected poses to avoid joint limit violations

#### 📍 Motion Sequence

1. Approach pick position
2. Move to pick
3. Close gripper
4. Lift object
5. Approach place position
6. Move to place
7. Open gripper
8. Retreat

---

### 📐 Defined Poses

| Stage          | Position (x, y, z) |
| -------------- | ------------------ |
| Approach Pick  | (0.30, 0.00, 0.25) |
| Pick           | (0.30, 0.00, 0.15) |
| Lift           | (0.30, 0.00, 0.35) |
| Approach Place | (0.32, 0.05, 0.25) |
| Place          | (0.32, 0.05, 0.15) |
| Retreat        | (0.32, 0.05, 0.30) |

---

### 🤏 Gripper Configuration

* **Open:** `[0.0]*6`
* **Close:** `[0.85]*6`

---

### 🚀 Launch Integration

File: `launch/test_xarm_pick_place.launch.py`

* Automatically spawns gripper trajectory controller
* Default:

  * `gripper_controller := xarm_gripper_traj_controller`
  * `controller_manager := /controller_manager`
* Runs pick-and-place node via `_robot_planner.launch.py` with gripper enabled

---

## ⚙️ Prerequisites

* ROS 2 (compatible distribution)
* MoveIt 2
* xArm7 ROS 2 / MoveIt configuration
* Gripper controller (`xarm_gripper_traj_controller`)

---

## 🛠 Build Instructions

```bash
colcon build --packages-select xarm_planner
source install/setup.bash
```

---

## ▶️ Running the System

### 🧪 Simulation (Fake Hardware)

```bash
ros2 launch xarm_planner xarm7_planner_fake.launch.py dof:=7 robot_type:=xarm add_gripper:=true
ros2 launch xarm_planner test_xarm_pick_place.launch.py dof:=7 robot_type:=xarm add_gripper:=true add_vacuum_gripper:=false
```

---

### 🌍 Gazebo Simulation

```bash
ros2 launch xarm_planner xarm7_planner_gazebo.launch.py dof:=7 robot_type:=xarm add_gripper:=true
ros2 launch xarm_planner test_xarm_pick_place.launch.py dof:=7 robot_type:=xarm add_gripper:=true add_vacuum_gripper:=false
```

---

### 🤖 Real Robot

> ⚠️ Ensure correct IP configuration and ROS 2 control parameters before running

```bash
ros2 launch xarm_planner xarm7_planner_realmove.launch.py dof:=7 robot_type:=xarm add_gripper:=true
ros2 launch xarm_planner test_xarm_pick_place.launch.py dof:=7 robot_type:=xarm add_gripper:=true add_vacuum_gripper:=false
```

---

## ⚠️ Notes & Troubleshooting

* If controllers are namespaced, override:

  ```bash
  controller_manager:=/xarm/controller_manager
  gripper_controller:=xarm_gripper_traj_controller
  ```

* If IK fails or robot hits joint limits:

  * Reduce **x/y reach**
  * Increase **z height**
  * Modify poses in:

    ```
    test/test_xarm_pick_place.cpp
    ```

* Use:

  ```bash
  add_vacuum_gripper:=false
  ```

  for finger gripper operation

---

## 📜 License

This project is based on the UFACTORY xArm ROS 2 framework.
All original code retains its respective license.

---

## 👤 Author

**Brian Kiprono**
Robotics | ROS 2 | Automation Systems

---

## 🔮 Future Work

* Vision-based object detection (OpenCV / AI)
* Dynamic pick-and-place targets
* Integration with mobile robots (AGV)
* Advanced task sequencing (MoveIt Task Constructor)

---

## ⭐ Acknowledgements

* UFACTORY for the xArm platform
* ROS 2 and MoveIt communities
