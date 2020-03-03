import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        node_name='msp_bridge_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='msp_bridge',
                node_plugin='acrobat::msp_bridge::MspBridge',
                node_name='msp_bridge',
                parameters=[{ 'device': "/dev/ttyUSB0", 'baudrate': 500000, 'imu_frequency': 100.0, 'motor_frequency': 100.0}]
            )
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
