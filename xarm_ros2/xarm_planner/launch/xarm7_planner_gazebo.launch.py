#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.cub@gmail.com>
# Modified by Brian Kiprono
# Changes: Added custom pick-and-place logic, updated motion planning, 
#          integrated ROS2 nodes and xarm_gripper_traj_controller spawner.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    
    # 1. Main robot moveit gazebo launch
    robot_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_gazebo.launch.py'
        ])),
        launch_arguments={
            'dof': '7',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'true',
            'add_gripper': 'true', # Must be true to load the gripper joints in Gazebo
        }.items(),
    )

    # 2. Gripper Controller Spawner
    # This activates the xarm_gripper_traj_controller so MoveIt can talk to it
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xarm_gripper_traj_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    return LaunchDescription([
        robot_moveit_gazebo_launch,
        gripper_controller_spawner
    ])