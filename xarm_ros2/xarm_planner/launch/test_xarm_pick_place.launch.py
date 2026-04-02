#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2026, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.cub@gmail.com>
# Modified by Brian Kiprono
# Changes: Added custom pick-and-place logic, updated motion planning, integrated ROS2 nodes

import json
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof')
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    gripper_controller = LaunchConfiguration('gripper_controller', default='xarm_gripper_traj_controller')
    controller_manager = LaunchConfiguration('controller_manager', default='/controller_manager')

    node_executable = 'test_xarm_pick_place'
    node_parameters = {}

    robot_planner_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', '_robot_planner.launch.py'])
        ),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': 'true',
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'robot_type': robot_type,
            'node_executable': node_executable,
            'node_parameters': json.dumps(node_parameters),
            'use_gripper_node': 'false',
        }.items(),
    )

  
    
    return [
        robot_planner_node_launch,
        
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
