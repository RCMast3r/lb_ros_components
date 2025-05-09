import os

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with multiple components."""

    # config = os.path.join(
    #   get_package_share_directory('lidar_bike_components'),
    #   'config',
    #   'config_local.yaml'
    #   )
    config = os.path.join('config', 'config_local.yaml') 
    container = ComposableNodeContainer(    
            name='test_cont',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='lidar_bike_components',
                    plugin='lidar_bike_calibration::CalibNode',
                    name='CalibNodeComponent',
                    parameters=[config]),
                ComposableNode(
                    package='rosbag2_transport',
                    plugin='rosbag2_transport::Player',
                    name='player',
                    parameters=[config]),
                ComposableNode(
                    package='foxglove_bridge',
                    plugin='foxglove_bridge::FoxgloveBridge',
                    name='fb',
                    parameters=[config])
            ],
            output='both',
    )

    return launch.LaunchDescription([container])