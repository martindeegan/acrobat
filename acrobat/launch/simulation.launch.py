import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    ros1_bridge_node = launch_ros.actions.Node(
        package='ros1_bridge',
        node_executable='dynamic_bridge',
        node_name='ros1_bridge_node',
        output='screen',
        emulate_tty=True
    )
    return launch.LaunchDescription([ros1_bridge_node])
