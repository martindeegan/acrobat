import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        node_name='arducam_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='arducam',
                node_plugin='acrobat::arducam::ArducamDriver',
                node_name='arducam_driver',
                parameters=[{
                    'config_name': 'camera_register_config.cfg', 
                    'camera_delay':18.5
                }]
            ),
            ComposableNode(
                package='arducam',
                node_plugin='acrobat::image_viewer::ImageViewer',
                node_name='image_viewer',
            )
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
