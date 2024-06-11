import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="pgo",
                namespace="pgo",
                executable="pgo_node",
                name="pgo_node",
                output="screen",
                parameters=[{"config_path": config_path.perform(launch.LaunchContext())}]
            )
        ]
    )
