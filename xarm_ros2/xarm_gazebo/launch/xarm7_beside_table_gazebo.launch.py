#!/usr/bin/env python3  # Use Python 3 from the active environment.
# Software License Agreement (BSD License)  # File is distributed under BSD terms.
#  # Spacer line in license header.
# Copyright (c) 2021, UFACTORY, Inc.  # Copyright owner and year.
# All rights reserved.  # Rights statement.
#  # Spacer line in license header.
# Author: Vinman <vinman.cub@gmail.com>
# Modified by Brian Kiprono
# Changes: Added custom pick-and-place logic, updated motion planning, integrated ROS2 nodes

from launch import LaunchDescription  # ROS 2 launch container returned by this file.
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument  # Launch actions used by this file.
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Loader for included Python launch files.
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir  # Runtime launch arguments and current file dir.

def generate_launch_description():  # Standard ROS 2 entry function for launch files.
    prefix = LaunchConfiguration('prefix', default='')  # Optional robot name prefix.
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')  # Hardware namespace for nodes/topics.
    limited = LaunchConfiguration('limited', default=False)  # Enable joint-limited model/control.
    effort_control = LaunchConfiguration('effort_control', default=False)  # Enable effort controller mode.
    velocity_control = LaunchConfiguration('velocity_control', default=False)  # Enable velocity controller mode.
    add_gripper = LaunchConfiguration('add_gripper', default=True)  # Attach standard gripper by default.
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)  # Optional vacuum gripper.

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)  # Optional RealSense D435i camera.

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)  # Optional custom end-effector geometry.
    geometry_type = LaunchConfiguration('geometry_type', default='box')  # Geometry primitive type.
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)  # Geometry mass in kg.
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)  # Geometry height parameter.
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)  # Geometry radius parameter.
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)  # Geometry length parameter.
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)  # Geometry width parameter.
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')  # Mesh file path when mesh geometry is used.
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')  # Mesh origin translation.
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')  # Mesh origin rotation (rpy).
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')  # TCP translation relative to mesh.
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')  # TCP rotation relative to mesh.

    # robot gazebo launch  # Include the shared Gazebo launch wrapper for robot beside table scene.
    # xarm_gazebo/launch/_robot_beside_table_gazebo.launch.py  # Target launch file being included.
    robot_gazobo_launch = IncludeLaunchDescription(  # Build include action for downstream launch file.
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/_robot_beside_table_gazebo.launch.py']),  # Resolve sibling launch file path.
        launch_arguments={  # Arguments forwarded to included launch file.
            'prefix': prefix,  # Forward prefix setting.
            'hw_ns': hw_ns,  # Forward hardware namespace.
            'limited': limited,  # Forward limited-model flag.
            'effort_control': effort_control,  # Forward effort control flag.
            'velocity_control': velocity_control,  # Forward velocity control flag.
            'add_gripper': add_gripper,  # Forward standard gripper flag.
            'add_vacuum_gripper': add_vacuum_gripper,  # Forward vacuum gripper flag.
            'dof': '7',  # Fixed to xArm7 degrees of freedom.
            'robot_type': 'xarm',  # Fixed robot family.
            'add_realsense_d435i': add_realsense_d435i,  # Forward camera flag.
            'add_other_geometry': add_other_geometry,  # Forward custom geometry flag.
            'geometry_type': geometry_type,  # Forward geometry type.
            'geometry_mass': geometry_mass,  # Forward geometry mass.
            'geometry_height': geometry_height,  # Forward geometry height.
            'geometry_radius': geometry_radius,  # Forward geometry radius.
            'geometry_length': geometry_length,  # Forward geometry length.
            'geometry_width': geometry_width,  # Forward geometry width.
            'geometry_mesh_filename': geometry_mesh_filename,  # Forward mesh file path.
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,  # Forward mesh origin xyz.
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,  # Forward mesh origin rpy.
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,  # Forward tcp xyz.
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,  # Forward tcp rpy.
        }.items(),  # Launch include expects iterable key/value pairs.
    )

    return LaunchDescription([  # Return all launch actions for execution.
        robot_gazobo_launch  # Run included robot Gazebo launch action.
    ])
