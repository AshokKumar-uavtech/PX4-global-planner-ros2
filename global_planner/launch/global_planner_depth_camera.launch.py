#!/usr/bin/env python

import os.path as osp

from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import \
    PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    tf2_static_pub_node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_depth_camera',
                arguments=['0', '0', '0',
                           '-1.57', '0', '-1.57',
                           'local_origin_odom', 'camera_frame'],
                # arguments=['0.9', '1.5', '0',
                #            '-1.57', '0', '-1.57',
                #            'local_origin_odom', 'camera_frame'],
                output='screen')

    tf2_static_pub_node2 = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_frd_ned',
                arguments=['0', '0', '0',
                           '1.57', '0', '3.14',
                          'base_frame', 'base_frame_ned'],
                output='screen')

    rviz2_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', '/home/user/git/global_planner_ws/src/PX4-global-planner-ros2/global_planner/resources/global_planner.rviz'])

    gp_params = {'frame_id': 'base_frame',
                 'world_path': '/home/user/git/global_planner_ws/src/PX4-global-planner-ros2/avoidance/sim/worlds/simple_obstacle.yaml'}
    
    gp_node = Node(package='global_planner',
                 executable='global_planner_node',
                 output='screen',
                 parameters=[gp_params])

    octomap_params = {'resolution': 1.0,
              'frame_id': 'base_frame',
              'base_frame_id': 'base_footprint',
              'height_map': True,
              'colored_map': False,
              'color_factor': 0.8,
              'filter_ground': False,
              'filter_speckles': False,
              'compress_map': True,
              'incremental_2D_projection': False,
              'sensor_model/max_range': 9.0,
              'sensor_model/hit': 0.9,
              'sensor_model/miss': 0.45,
              'sensor_model/min': 0.01,
              'sensor_model/max': 0.99,
              'pointcloud_max_x': 100.0,
              'pointcloud_max_y': 100.0,
              'pointcloud_max_z': 100.0,
              'pointcloud_min_x': -100.0,
              'pointcloud_min_y': -100.0,
              'pointcloud_min_z': -100.0,
              'occupancy_min_z': 0.5,
              'color/r': 0.0,
              'color/g': 0.0,
              'color/b': 1.0,
              'color/a': 1.0,
              'color_free/r': 0.0,
              'color_free/g': 0.0,
              'color_free/b': 1.0,
              'color_free/a': 1.0,
              'publish_free_space': False,
    }

    # octomap_remap = [('cloud_in', '/camera/points')]
    
    octomap_node = Node(package='octomap_server2',
                 executable='octomap_server',
                 output='log',
                 # remappings=octomap_remap,
                 parameters=[octomap_params])
    return LaunchDescription([tf2_static_pub_node, tf2_static_pub_node2, octomap_node, gp_node, rviz2_node])
