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
            parameters=[{
                'use_arducam': True,
                'use_image_viewer': False,
                'use_msp_bridge': True
            },
                #  Arducam driver configuration
                {
                'config_name': 'camera_register_config.cfg',
                'capture_frequency': 50.0,
            },
                #  MSP Bridge configuration
                {
                'device': '/dev/acrobat/fc',
                'baudrate': 500000,
                'imu_frequency': 100.0,
                'motor_frequency': 100.0
            }
            ]
        )
    ])
