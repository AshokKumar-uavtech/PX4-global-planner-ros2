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

    camera_frame_name = 'camera_frame'  # Realsense camera frame name

    tf2_static_pub_node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_depth_camera',
                arguments=['0', '0', '0',
                           '-1.57', '0', '-1.57',
                           'local_origin_odom', camera_frame_name],
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
                arguments=['-d', '/home/karidrone/git/global_planner_ws/src/PX4-global-planner-ros2/global_planner/resources/global_planner.rviz'])

    gp_params = {'frame_id': 'base_frame',
                 'agent_number': 14,
                 'position_mode': 'local_position',
                 'world_path': '/home/user/git/global_planner_ws/src/PX4-global-planner-ros2/avoidance/sim/worlds/simple_obstacle.yaml',
                 'pointcloud_topics': ['/camera/depth/color/points'],
                 'start_pos_x': 0.0,
                 'start_pos_y': 0.0,
                 'start_pos_z': 3.0,
                 'min_altitude': 1,
                 'max_altitude': 5,
                 'max_cell_risk': 0.5,
                 'smooth_factor': 10.0,
                 'vert_to_hor_cost': 1.0,
                 'risk_factor': 500.0,
                 'neighbor_risk_flow': 1.0,
                 'explore_penalty': 0.005,
                 'up_cost': 3.0,
                 'down_cost': 1.0,
                 'search_time': 0.5,
                 'min_overestimate_factor': 1.03,
                 'max_overestimate_factor': 2.0,
                 'risk_threshold_risk_based_speedup': 0.5,
                 'default_speed': 1.0,
                 'max_speed': 3.0,
                 'max_iterations': 2000,
                 'goal_is_blocked': False,
                 'current_cell_blocked': False,
                 'goal_must_be_free': True,
                 'use_current_yaw': True,
                 'use_risk_heuristics': True,
                 'use_speedup_heuristics': True,
                 'use_risk_based_speedup': True}
    
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
              'sensor_model/max_range': 6.0,
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
    return LaunchDescription([tf2_static_pub_node, tf2_static_pub_node2, octomap_node, gp_node]) #, rviz2_node])
