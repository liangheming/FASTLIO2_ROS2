#!/usr/bin/env python3
import os
import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Launch file for localizer with RoboSense Airy LiDAR
    Uses lio_airy.yaml config for fastlio2
    Includes airy_tf_tree.launch.py for TF
    """

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock for rosbag playback'
    )

    map_pcd_arg = DeclareLaunchArgument(
        'map_pcd',
        default_value='/home/steve/map_data/map_geoscan/B1/20251228_152049/global_map.pcd',
        description='Path to global map PCD file'
    )

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("localizer"), "rviz", "localizer.rviz"]
    )
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer_fastlio2.yaml"]
    )

    # Use lio_airy.yaml for RoboSense Airy
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio_airy.yaml"]
    )

    # Include airy_tf_tree.launch.py for TF
    # TF tree: map -> odom -> base_link -> imu_link -> rslidar
    tf_launch_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "tf", "airy_tf_tree.launch.py"]
    )
    airy_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_launch_path),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    return launch.LaunchDescription(
        [
            use_sim_time_arg,
            map_pcd_arg,

            # TF tree from airy_tf_tree.launch.py
            airy_tf_launch,

            # Global map server
            launch_ros.actions.Node(
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
            ),

            # FAST-LIO2 node with Airy config
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[
                    {"config_path": lio_config_path.perform(launch.LaunchContext())},
                    {"use_sim_time": LaunchConfiguration('use_sim_time')}
                ],
            ),

            # Localizer node
            launch_ros.actions.Node(
                package="localizer",
                namespace="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[
                    {
                        "config_path": localizer_config_path.perform(
                            launch.LaunchContext()
                        )
                    },
                    {"use_sim_time": LaunchConfiguration('use_sim_time')}
                ],
            ),

            # RViz2
            launch_ros.actions.Node(
                package="rviz2",
                namespace="localizer",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
                parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}]
            ),
        ]
    )
