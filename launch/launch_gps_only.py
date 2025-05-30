import os

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with multiple components."""

    # config = os.path.join(
    #   get_package_share_directory('mcap_writer_component'),
    #   'config',
    #   'config_local.yaml'
    #   )
    
    config = os.path.join(
    #   get_package_share_directory('mcap_writer_component'),
      'config',
      'config_local.yaml'
      )
    
    container = ComposableNodeContainer(    
            name='test_cont',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='mcap_writer_component',
                    plugin='mcap_writer_component::MCAPRecorder',
                    name='mcap_boi',
                    parameters=[config]),
                    # extra_arguments=[{'use_intra_process_comms': True}]),
                    
                # ComposableNode(
                #     package='ouster_ros',
                #     plugin='ouster_ros::OusterDriver',
                #     name='driver',
                #     # extra_arguments=[{'use_intra_process_comms': True}],
                #     parameters=[config]),
                # ComposableNode(
                #     package='v4l2_camera',
                #     plugin='v4l2_camera::V4L2Camera',
                #     name='v4l2_camera',
                #     parameters=[config])
                ComposableNode(
                    package='ublox_gps',
                    plugin='ublox_node::UbloxNode',
                    name='ublox_node',
                    parameters=[config])
            ],
            output='both',
    )

    return launch.LaunchDescription([container])

# TODO ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video1" -p image_size:=[1920,1080] -p pixel_format:="MJPG"