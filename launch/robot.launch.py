from  launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_camera_driver',
            node_namespace='',
            node_executable='usb_camera_driver_node',
            node_name='camera'
        ),
        Node(
            package='opt_flow_nav',
            node_namespace='',
            node_executable='analyze_optical_flow.py',
            node_name='opt_flow_nav'
        )
    ])