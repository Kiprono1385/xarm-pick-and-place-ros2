cmake_minimum_required(VERSION 3.8)
project(my_custom_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- ADD C++ EXECUTABLES ---
add_executable(pick_and_place_gazebo src/pick_and_place_gazebo.cpp)
ament_target_dependencies(pick_and_place_gazebo
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  linkattacher_msgs
  xarm_msgs
  yaml-cpp
)

add_executable(pick_and_place_real src/pick_and_place_real.cpp)
ament_target_dependencies(pick_and_place_real
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  xarm_msgs
  yaml-cpp
)

# --- INSTALL TARGETS ---
install(TARGETS
  pick_and_place_gazebo
  pick_and_place_real
  DESTINATION lib/${PROJECT_NAME}
)

# --- INSTALL DIRECTORIES ---
install(DIRECTORY launch object_scenes config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()