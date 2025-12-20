import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # RViz config (use the one in pgo package if exists; else use fastlio2 rviz)
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("pgo"), "rviz", "pgo.rviz"]
    )

    # YAML configs
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    return launch.LaunchDescription([
        # LIO
        launch_ros.actions.Node(
            package="fastlio2",
            namespace="fastlio2",
            executable="lio_node",
            name="lio_node",
            output="screen",
            parameters=[
                {"config_path": lio_config_path},
                {"use_sim_time": True},
            ],
            remappings=[
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
            ],
        ),

        # PGO
        launch_ros.actions.Node(
            package="pgo",
            executable="pgo_node",
            name="pgo_node",
            output="screen",
            parameters=[
                {"config_path": pgo_config_path},
                {"use_sim_time": True},
            ],
        ),

        # RViz
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=["-d", rviz_cfg],
        ),
    ])
