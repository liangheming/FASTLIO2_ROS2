import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution([FindPackageShare("hba"), "rviz", "hba.rviz"])
    config_path = PathJoinSubstitution([FindPackageShare("hba"), "config", "hba.yaml"])

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="hba",
                namespace="hba",
                executable="hba_node",
                name="hba",
                output="screen",
                parameters=[
                    {"config_path": config_path.perform(launch.LaunchContext())}
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="hba",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            ),
        ]
    )
