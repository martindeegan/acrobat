import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    container = ComposableNodeContainer(
        node_name='acrobat_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='msp_bridge',
                node_plugin='acrobat::msp_bridge::MspBridge',
                node_name='msp_bridge',
                parameters=[{ 'device': "/dev/ttyUSB0", 'baudrate': 500000, 'imu_frequency': 100.0, 'motor_frequency': 100.0}]
            ),
            ComposableNode(
                package='arducam',
                node_plugin='acrobat::arducam::ArducamDriver',
                node_name='arducam_driver',
                parameters=[{'config_name': 'camera_register_config.cfg'}]
            )
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
