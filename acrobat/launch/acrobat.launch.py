import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='acrobat', 
            node_executable='acrobat_executor', 
            output='screen', 
            emulate_tty=True,
            parameters=[{'config_name': 'camera_register_config.cfg'},{'camera_delay':18.5}]
        )
    ])
