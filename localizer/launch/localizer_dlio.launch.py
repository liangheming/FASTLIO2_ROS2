#!/usr/bin/env python3
"""
Localizer launch file using DLIO (Direct LiDAR-Inertial Odometry)
for RoboSense Airy LiDAR
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
    dlio_pkg = FindPackageShare("dlio")
    map_server_pkg = FindPackageShare("map_server")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock for rosbag playback'
    )

    map_pcd_arg = DeclareLaunchArgument(
        'map_pcd',
        default_value='/home/steve/map_data/map_geoscan/B1/20251225_191653/global_map.pcd',
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

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/rslidar_imu_data',
        description='IMU topic name'
    )

    # Config paths
    localizer_config_path = PathJoinSubstitution(
        [localizer_pkg, "config", "localizer_dlio.yaml"]
    )
    rviz_cfg = PathJoinSubstitution(
        [localizer_pkg, "rviz", "localizer.rviz"]
    )
    dlio_yaml_path = PathJoinSubstitution(
        [dlio_pkg, 'config', 'airy.yaml']
    )
    dlio_params_yaml_path = PathJoinSubstitution(
        [dlio_pkg, 'config', 'params.yaml']
    )

    # Include TF tree launch file
    tf_launch_path = PathJoinSubstitution(
        [localizer_pkg, "tf", "airy_tf_tree.launch.py"]
    )
    airy_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_launch_path),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # DLIO Odometry Node
    dlio_odom_node = launch_ros.actions.Node(
        package='dlio',
        executable='direct_odometry_node',
        name='direct_odometry_node',
        output='screen',
        parameters=[
            dlio_yaml_path,
            dlio_params_yaml_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('imu', LaunchConfiguration('imu_topic')),
            ('pointcloud', LaunchConfiguration('pointcloud_topic')),
            ('odom', 'direct_odometry/odom'),
            ('pose', 'direct_odometry/pose'),
            ('path', 'direct_odometry/path'),
            ('kf_pose', 'direct_odometry/keyframes'),
            ('kf_cloud', 'direct_odometry/pointcloud/keyframe'),
            ('deskewed', 'direct_odometry/pointcloud/deskewed'),
            ('body_cloud', 'direct_odometry/body_cloud'),
            ('sparse_map', 'direct_odometry/sparse_map'),
        ],
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
        imu_topic_arg,

        # TF tree
        airy_tf_launch,

        # Nodes
        dlio_odom_node,
        globalmap_server_node,
        localizer_node,
        rviz_node,
    ])
