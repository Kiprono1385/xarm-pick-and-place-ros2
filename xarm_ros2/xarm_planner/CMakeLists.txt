cmake_minimum_required(VERSION 3.5)
project(xarm_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
    -Wno-sign-compare
    -Wno-unused-parameter 
    -Wno-unused-variable
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)

set(dependencies 
  "rclcpp" 
  "std_msgs" 
  "geometry_msgs" 
  "moveit_msgs" 
  "xarm_msgs" 
  "moveit" 
  "moveit_core"
  "linkattacher_msgs" 
  "moveit_task_constructor_core" 
  "moveit_ros_planning_interface"
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  /opt/ros/humble/include
)

add_library(xarm_planner SHARED
  src/xarm_planner.cpp
)
ament_target_dependencies(xarm_planner 
  ${dependencies}
)

add_executable(xarm_planner_node src/xarm_planner_node.cpp)
ament_target_dependencies(xarm_planner_node
  ${dependencies}
)
target_link_libraries(xarm_planner_node 
  xarm_planner
  ${ament_LIBRARIES}
)

add_executable(xarm_gripper_planner_node src/xarm_gripper_planner_node.cpp)
ament_target_dependencies(xarm_gripper_planner_node
  ${dependencies}
)
target_link_libraries(xarm_gripper_planner_node 
  xarm_planner
  ${ament_LIBRARIES}
)

add_executable(test_xarm_planner_api_joint test/test_xarm_planner_api_joint.cpp)
ament_target_dependencies(test_xarm_planner_api_joint
  ${dependencies}
)
target_link_libraries(test_xarm_planner_api_joint 
  xarm_planner
  ${ament_LIBRARIES}
)

add_executable(test_dual_xarm_planner_api_joint test/test_dual_xarm_planner_api_joint.cpp)
ament_target_dependencies(test_dual_xarm_planner_api_joint
  ${dependencies}
)
target_link_libraries(test_dual_xarm_planner_api_joint 
  xarm_planner
  ${ament_LIBRARIES}
)

add_executable(test_xarm_planner_api_pose test/test_xarm_planner_api_pose.cpp)
ament_target_dependencies(test_xarm_planner_api_pose
  ${dependencies}
)
target_link_libraries(test_xarm_planner_api_pose 
  xarm_planner
  ${ament_LIBRARIES}
)

add_executable(test_xarm_gripper_planner_api_joint test/test_xarm_gripper_planner_api_joint.cpp)
ament_target_dependencies(test_xarm_gripper_planner_api_joint
  ${dependencies}
)
target_link_libraries(test_xarm_gripper_planner_api_joint 
  xarm_planner
  ${ament_LIBRARIES}
)

add_executable(test_xarm_planner_client_joint test/test_xarm_planner_client_joint.cpp)
ament_target_dependencies(test_xarm_planner_client_joint
  ${dependencies}
)

add_executable(test_xarm_planner_client_pose test/test_xarm_planner_client_pose.cpp)
ament_target_dependencies(test_xarm_planner_client_pose
  ${dependencies}
)

add_executable(test_xarm_gripper_planner_client_joint test/test_xarm_gripper_planner_client_joint.cpp)
ament_target_dependencies(test_xarm_gripper_planner_client_joint
  ${dependencies}
)

add_executable(test_lite_gripper_realmove test/test_lite_gripper_realmove.cpp)
ament_target_dependencies(test_lite_gripper_realmove
  ${dependencies}
)

# --- Pick and Place Executable (Explicit Target Linking) ---
add_executable(test_xarm_pick_place test/test_xarm_pick_place.cpp)
ament_target_dependencies(test_xarm_pick_place
  rclcpp
  std_msgs
  geometry_msgs
  moveit_msgs
  xarm_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  linkattacher_msgs
)
target_link_libraries(test_xarm_pick_place 
  xarm_planner
  ${ament_LIBRARIES}
)
# -----------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS 
  xarm_planner
  xarm_planner_node
  xarm_gripper_planner_node
  test_xarm_planner_api_joint
  test_dual_xarm_planner_api_joint
  test_xarm_planner_api_pose
  test_xarm_gripper_planner_api_joint
  test_xarm_planner_client_joint
  test_xarm_planner_client_pose
  test_xarm_gripper_planner_client_joint
  test_lite_gripper_realmove
  test_xarm_pick_place
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()