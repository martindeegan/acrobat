import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        node_name='serial_bridge_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='serial_bridge',
                node_plugin='acrobat::serial_bridge::SerialBridge',
                node_name='serial_bridge',
                parameters=[{ 'device': '/dev/ttyUSB0' }]
            )
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
