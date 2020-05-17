from  launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_camera_driver',
            node_executable='usb_camera_driver_node',
            node_name='camera',
            parameters=[Path(get_package_share_directory("opt_flow_nav"), "usb_camera_driver.yaml")]
        ),
        Node(
            package='rqt_image_view',
            node_executable='rqt_image_view',
            node_name='rqt_image_view',
        )
        #Node(
        #    package='opt_flow_nav',
        #    node_namespace='/',
        #    node_executable='analyze_optical_flow',
        #    node_name='opt_flow_nav'
        #)
    ])