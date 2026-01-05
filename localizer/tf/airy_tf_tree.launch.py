#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    统一的TF管理launch文件
    定义机器人所有静态坐标系关系
    """
    
    # ============================================
    # 外参定义区域 - 集中管理所有外参
    # ============================================
    
    # base_link -> imu_link 外参
    # 修改为让 rslidar 竖直安装（rslidar 沿其 Y 轴旋转 90°）
    EXTRINSIC_BASE_TO_IMU = {
        'x': '0.0',
        'y': '0.0',
        'z': '0.0',
        'roll': '3.1415',    # 180度
       #'roll': '1.5708',   # 90度 (从 180度 改为 90度)
        'pitch': '0.0',
        'yaw': '-1.5708',      # -90度
    }
    
    # imu_link -> rslidar 外参（标定值或理论值）
    EXTRINSIC_IMU_TO_LIDAR = {
        'x': '0.004250',
        'y': '0.004180',
        'z': '-0.004460',
        'qx': '0.709757',    # 四元数
        'qy': '-0.704427',
        'qz': '0.004992',
        'qw': '-0.001454',
    }
    
    # ============================================
    # Launch参数
    # ============================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # rosbag回放使用true (记得播放时加 --clock 参数)
        description='Use simulation clock (true for rosbag with --clock, false for real sensors)'
    )
    
    # ============================================
    # TF树结构（静态部分）
    # map -> odom -> base_link -> imu_link -> rslidar
    #                                      -> rslidar_optimized (动态，由SLAM发布)
    # ============================================
    
    # 1. map -> odom (初始静态，SLAM启动后会动态发布)
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 2. odom -> base_link (初始静态，SLAM启动后会动态发布)
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 3. base_link -> imu_link (固定机械关系，永远不变)
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '--x', EXTRINSIC_BASE_TO_IMU['x'],
            '--y', EXTRINSIC_BASE_TO_IMU['y'],
            '--z', EXTRINSIC_BASE_TO_IMU['z'],
            '--roll', EXTRINSIC_BASE_TO_IMU['roll'],
            '--pitch', EXTRINSIC_BASE_TO_IMU['pitch'],
            '--yaw', EXTRINSIC_BASE_TO_IMU['yaw'],
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 4. imu_link -> rslidar (初始标定值，固定)
    static_tf_imu_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu_to_lidar',
        arguments=[
            EXTRINSIC_IMU_TO_LIDAR['x'],
            EXTRINSIC_IMU_TO_LIDAR['y'],
            EXTRINSIC_IMU_TO_LIDAR['z'],
            EXTRINSIC_IMU_TO_LIDAR['qx'],
            EXTRINSIC_IMU_TO_LIDAR['qy'],
            EXTRINSIC_IMU_TO_LIDAR['qz'],
            EXTRINSIC_IMU_TO_LIDAR['qw'],
            'imu_link',
            'rslidar'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 注意：优化后的外参 imu_link -> rslidar_optimized 由SLAM节点动态发布
    # SLAM节点会发布：tf: imu_link -> rslidar_optimized
    # 这样可以同时保留原始标定值和优化值，方便对比
    
    return LaunchDescription([
        # Launch参数
        use_sim_time_arg,
        
        # TF发布节点
        static_tf_map_to_odom,
        static_tf_odom_to_base,
        static_tf_base_to_imu,
        static_tf_imu_to_lidar,
    ])