from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import SetLaunchConfiguration, DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
import launch
from ament_index_python import get_package_share_directory

import os
from typing import Dict

dataset_uri = '/home/martin/fpv_datasets/uzh_fpv/outdoor_forward_1_snapdragon_with_gt.bag'
module_toggles = {
    'sensors': {
        'arducam': False,
        'msp_bridge': False,
        'vectornav': False
    },
    'autonomy': {
        'visual_odometry': True
    },
    'utilities': {
        'image_viewer': False
    }
}

vio_params = {
    'camera_topic': '/snappy_cam/stereo_r',
    'imu_topic': '/snappy_imu',
    'ground_truth_transform_topic': '/vrpn_client/raw_transform',
    'ground_truth_pose_topic': '/groundtruth/pose',
    'display': True
}

rviz_reference_frame = 'ground_truth'


def get_toggle(name: str):
    return LaunchConfiguration('enable_' + name)


def get_acrobat_sensor_nodes(sensor_toggles: Dict[str, bool]):

    arducam_node = ComposableNode(
        package='arducam',
        node_plugin='ArducamDriver',
        node_name='arducam_driver',
        parameters=[{
            'config_name': 'camera_register_config.cfg'
        }]
    )

    msp_bridge_node = ComposableNode(
        package='msp_bridge',
        node_plugin='acrobat::msp_bridge::MspBridge',
        node_name='msp_bridge',
        parameters=[{
            'device': "/dev/ttyUSB0",
            'baudrate': 500000,
            'imu_frequency': 100.0,
            'motor_frequency': 100.0
        }],
    )

    vectornav_node = ComposableNode(
        package='vectornav',
        node_plugin='vn_ros::VectorNavNode',
        node_name='vectornav_node',
        parameters=[{
            "sensor_port": "/dev/ttyUSB0",
            "baudrate": 921600,
            "sample_rate": 200,
            "topic": "/acrobat/imu",
            "frame_id": "acrobat_imu",
            "gyroscope_variance": 1e-3,
            "accelerometer_variance": 1e-3
        }],
    )

    sensor_nodes = []
    if sensor_toggles['arducam']:
        sensor_nodes.append(arducam_node)
    if sensor_toggles['msp_bridge']:
        sensor_nodes.append(msp_bridge_node)
    if sensor_toggles['vectornav']:
        sensor_nodes.append(vectornav_node)

    return sensor_nodes


def get_acrobat_autonomy_nodes(autonomy_toggles: Dict[str, bool]):
    vio_node = ComposableNode(
        package='acrobat_localization',
        node_plugin='acrobat::localization::VisualOdometry',
        node_name='acrobat_vio',
        parameters=[{
        }],
    )

    autonomy_nodes = []
    if autonomy_toggles['visual_odometry']:
        autonomy_nodes.append(vio_node)

    return autonomy_nodes


def get_acrobat_utility_nodes(utility_toggles: Dict[str, bool]):
    return []


def generate_launch_description():
    description = LaunchDescription()

    rviz_config_dir = os.path.join(
        get_package_share_directory('acrobat'),
        'rviz',
        'acrobat.rviz')

    # Launch RViz2
    description.add_action(Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz',
        arguments=['-d', rviz_config_dir, '-f', rviz_reference_frame],
        output='screen',
        emulate_tty=True
    ))

    description.add_action(
        LogInfo(msg=["===================================="]))
    for modules in module_toggles.values():
        for module_name, enabled in modules.items():
            description.add_action(
                LogInfo(msg=[module_name, ': ', "Enabled" if enabled else "Disabled"]))
    description.add_action(
        LogInfo(msg=["===================================="]))

    description.add_action(DeclareLaunchArgument(name='enable_rosbag_logging',
                                                 default_value='True', description='Enable rosbag2 logging'))
    description.add_action(DeclareLaunchArgument(name='enable_ros1_bridge',
                                                 default_value='True', description='Enable ros1 bridge'))
    description.add_action(DeclareLaunchArgument(name='enable_rviz',
                                                 default_value='True', description='Enable rviz2 gui'))
    # TODO remove
    vio_node = Node(
        package='acrobat_localization',
        node_executable='acrobat_vio',
        node_name='acrobat_vio',
        node_namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[vio_params]
    )
    description.add_action(vio_node)

    sensor_nodes = get_acrobat_sensor_nodes(module_toggles['sensors'])
    autonomy_nodes = get_acrobat_autonomy_nodes(module_toggles['autonomy'])
    utility_nodes = get_acrobat_utility_nodes(module_toggles['utilities'])

    compostable_nodes = sensor_nodes + autonomy_nodes + utility_nodes
    # TODO remove
    compostable_nodes = []

    acrobat_node_container = ComposableNodeContainer(
        node_name='acrobat_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=compostable_nodes,
        output='screen',
        emulate_tty=True,
    )

    # # Rosbag2 Logging
    # # Add more topics here
    # topics = ['/acrobat/imu', '/acrobat/camera', '/acrobat/fc_imu']
    description.add_action(ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-s', 'rosbag_v2', dataset_uri],
        name='rosbag2',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(get_toggle('rosbag_logging')))
    )

    return description
