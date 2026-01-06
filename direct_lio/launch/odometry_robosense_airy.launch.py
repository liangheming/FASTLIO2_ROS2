#
#   Copyright (c)     
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('dlio')

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='true')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/rslidar_points')
    imu_topic = LaunchConfiguration('imu_topic', default='/rslidar_imu_data')

    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )

    # Load parameters
    dlio_yaml_path = PathJoinSubstitution([current_pkg, 'config', 'airy.yaml'])
    dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'config', 'params.yaml'])

    # DLIO Odometry Node
    dlio_odom_node = Node(
        package='dlio',
        executable='direct_odometry_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('imu', imu_topic),
            ('pointcloud', pointcloud_topic),
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

    # Include TF tree launch file from localizer package
    localizer_pkg = FindPackageShare('localizer')
    tf_tree_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([localizer_pkg, 'tf', 'casbot_tf_tree.launch.py'])
        ])
    )

    # RViz node
    rviz_config_path = PathJoinSubstitution([current_pkg, 'rviz', 'odometry.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        tf_tree_launch,
        dlio_odom_node,
        rviz_node
    ])
