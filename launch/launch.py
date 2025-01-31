import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
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
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='ouster_ros',
                    plugin='ouster_ros::OusterDriver',
                    name='driver',
                    extra_arguments=[{'use_intra_process_comms': True}])
            ],
            output='both',
    )

    return launch.LaunchDescription([container])