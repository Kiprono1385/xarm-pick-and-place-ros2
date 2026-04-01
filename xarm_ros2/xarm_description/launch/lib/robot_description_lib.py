#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.cub@gmail.com>
# Modified by Brian Kiprono
# Changes: Added custom pick-and-place logic, updated motion planning, integrated ROS2 nodes

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def get_xacro_file_content(
    xacro_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']), 
    arguments={}):
    command = [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
        ' '
    ]
    if arguments and isinstance(arguments, dict):
        for key, val in arguments.items():
            command.extend([
                '{}:='.format(key),
                val,
                ' '
            ])
    return Command(command)
