#!/usr/bin/env python3
"""
Localizer launch file using GLIM (Global LiDAR-Inertial Mapping) odometry
for RoboSense Airy LiDAR

Prerequisites:
  Start GLIM odometry first in another terminal:
    ros2 launch glim_ros glim_odometry.launch.py

GLIM outputs (subscribed by this localizer):
  - /glim_ros/pose (PoseStamped)
  - /rslidar_points (PointCloud2, raw input)

Workflow:
  1. GLIM provides odometry (started separately)
  2. RViz publishes /initialpose
  3. manual_initialization_node refines initial pose, publishes /refined_initialpose
  4. localizer_node receives refined pose and starts tracking
"""
import os
import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    # Package paths
    localizer_pkg = FindPackageShare("localizer")
    map_server_pkg = FindPackageShare("map_server")
    manual_init_pkg = FindPackageShare("manual_initialization")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock for rosbag playback'
    )

    map_pcd_arg = DeclareLaunchArgument(
        'map_pcd',
       #default_value='/home/steve/map_data/map_geoscan/B1/20251225_191653/global_map.pcd',
       #default_value='/home/steve/map_data/map_geoscan/103/global_map.pcd',
        default_value='/home/steve/map_data/map_geoscan/1F/20251226_105050/global_map.pcd',
        description='Path to global map PCD file'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/rslidar_points',
        description='Pointcloud topic name'
    )

    # Config paths
    localizer_config_path = PathJoinSubstitution(
        [localizer_pkg, "config", "localizer_glim.yaml"]
    )
    rviz_cfg = PathJoinSubstitution(
        [localizer_pkg, "rviz", "localizer.rviz"]
    )
    manual_init_config_path = PathJoinSubstitution(
        [manual_init_pkg, 'config', 'manual_initialization.yaml']
    )

    # Include TF tree launch file
    tf_launch_path = PathJoinSubstitution(
        [localizer_pkg, "tf", "casbot_tf_tree.launch.py"]
    )
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_launch_path),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Global map server
    globalmap_server_node = launch_ros.actions.Node(
        package="map_server",
        executable="globalmap_server_node",
        name="globalmap_server",
        output="screen",
        parameters=[
            {"globalmap_pcd": LaunchConfiguration('map_pcd')},
            {"downsample_resolution": 0.2},
            {"frame_id": "map"},
            {"globalmap_topic": "/globalmap"},
            {"use_sim_time": LaunchConfiguration('use_sim_time')}
        ],
    )

    # Manual Initialization Node
    manual_init_node = launch_ros.actions.Node(
        package='manual_initialization',
        executable='manual_initialization_node',
        name='manual_initialization_node',
        output='screen',
        parameters=[
            manual_init_config_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('initialpose', '/initialpose'),
            ('cloud', LaunchConfiguration('pointcloud_topic')),
            ('map', '/globalmap'),
            ('refined_initialpose', '/refined_initialpose'),
            ('aligned_cloud', '/aligned_cloud'),
        ]
    )

    # Localizer node
    localizer_node = launch_ros.actions.Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {"config_path": localizer_config_path.perform(launch.LaunchContext())},
            {"use_sim_time": LaunchConfiguration('use_sim_time')}
        ],
    )

    # RViz2
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        namespace="localizer",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return launch.LaunchDescription([
        use_sim_time_arg,
        map_pcd_arg,
        rviz_arg,
        pointcloud_topic_arg,

        # TF tree
        tf_launch,

        # Nodes (GLIM started separately)
        globalmap_server_node,
        manual_init_node,
        localizer_node,
        rviz_node,
    ])
