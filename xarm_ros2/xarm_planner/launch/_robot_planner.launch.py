#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.cub@gmail.com>
# Modified by Brian Kiprono
# Changes: Added custom pick-and-place logic, updated motion planning, integrated ROS2 nodes
#          Added planning_pipelines (OMPL) params to fix CHOMP fallback/segfault

import yaml
import json
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    moveit_config_dump = LaunchConfiguration('moveit_config_dump', default='')
    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader) if moveit_config_dump else {}

    if not moveit_config_dict:
        moveit_config = MoveItConfigsBuilder(
            context=context,
            # controllers_name=controllers_name,
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            hw_ns=hw_ns,
            limited=limited,
            effort_control=effort_control,
            velocity_control=velocity_control,
            model1300=model1300,
            robot_sn=robot_sn,
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            mesh_suffix=mesh_suffix,
            kinematics_suffix=kinematics_suffix,
            # ros2_control_plugin=ros2_control_plugin,
            # ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
            add_vacuum_gripper=add_vacuum_gripper,
            add_bio_gripper=add_bio_gripper,
            add_realsense_d435i=add_realsense_d435i,
            add_d435i_links=add_d435i_links,
            add_other_geometry=add_other_geometry,
            geometry_type=geometry_type,
            geometry_mass=geometry_mass,
            geometry_height=geometry_height,
            geometry_radius=geometry_radius,
            geometry_length=geometry_length,
            geometry_width=geometry_width,
            geometry_mesh_filename=geometry_mesh_filename,
            geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
            geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
            geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
            geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
        ).to_moveit_configs()
        moveit_config_dict = moveit_config.to_dict()

    # --- DEBUG: print available keys so we can confirm exact pipeline key names ---
    print(">>> MoveIt config dict keys:", list(moveit_config_dict.keys()))

    move_group_interface_params = {
        'robot_description': moveit_config_dict['robot_description'],
        'robot_description_semantic': moveit_config_dict['robot_description_semantic'],
        'robot_description_kinematics': moveit_config_dict['robot_description_kinematics'],
    }

    # --- Add planning pipeline params (fixes OMPL not loading -> CHOMP fallback/segfault) ---
    # Grab every planning-pipeline-related key that exists in the dict, defensively,
    # since exact key names can vary slightly by moveit_configs_utils/uf_ros_lib version.
    for key in (
        'robot_description_planning',
        'planning_pipelines',
        'move_group_capabilities',
        'ompl',
        'chomp',
        'planning_plugin',
        'planning_scene_monitor',
    ):
        if key in moveit_config_dict:
            move_group_interface_params[key] = moveit_config_dict[key]

    # Explicit fallback: if 'planning_pipelines' wasn't in the dict but individual
    # pipeline configs were exposed as separate top-level keys, this ensures the
    # default pipeline is still set to ompl so PipelinePlanner("ompl") resolves correctly.
    if 'planning_pipelines' not in move_group_interface_params:
        move_group_interface_params['planning_pipelines'] = {
            'pipeline_names': ['ompl'],
            'default_planning_pipeline': 'ompl',
        }

    node_executable = LaunchConfiguration('node_executable', default='xarm_planner_node')
    node_name = LaunchConfiguration('node_name', default=node_executable)
    node_parameters = LaunchConfiguration('node_parameters', default={})
    use_gripper_node = LaunchConfiguration('use_gripper_node', default=add_gripper)

    try:
        xarm_planner_parameters = json.loads(node_parameters.perform(context))
    except:
        xarm_planner_parameters = {}

    xarm_planner_node = Node(
        name=node_name,
        package='xarm_planner',
        executable=node_executable,
        output='screen',
        parameters=[
            move_group_interface_params,
            {
                'robot_type': robot_type,
                'dof': dof,
                'prefix': prefix
            },
            xarm_planner_parameters,
        ],
    )

    nodes = [
        xarm_planner_node
    ]
    if robot_type.perform(context) != 'lite' and add_gripper.perform(context) in ('True', 'true') and use_gripper_node.perform(context) in ('True', 'true'):
        planning_group = 'uf850_gripper' if robot_type.perform(context) == 'uf850' else 'xarm_gripper'
        xarm_gripper_planner_node = Node(
            name='xarm_gripper_planner_node',
            package='xarm_planner',
            executable='xarm_gripper_planner_node',
            output='screen',
            parameters=[
                move_group_interface_params,
                {'PLANNING_GROUP': planning_group},
            ],
        )
        nodes.append(xarm_gripper_planner_node)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])