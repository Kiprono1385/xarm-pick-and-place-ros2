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
find_package(trajectory_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# Check for open62541 library via pkg-config
pkg_check_modules(OPEN62541 REQUIRED open62541)

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
target_include_directories(pick_and_place_real PUBLIC
  ${OPEN62541_INCLUDE_DIRS}
)
ament_target_dependencies(pick_and_place_real
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  xarm_msgs
  yaml-cpp
)
target_link_libraries(pick_and_place_real ${OPEN62541_LIBRARIES})

add_executable(ghost_relay_internal src/ghost_relay_internal.cpp)
ament_target_dependencies(ghost_relay_internal rclcpp moveit_msgs trajectory_msgs)

# --- INSTALL TARGETS ---
install(TARGETS
  pick_and_place_gazebo
  pick_and_place_real
  ghost_relay_internal
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