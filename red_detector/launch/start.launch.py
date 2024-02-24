from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    red_detector = Node(
        package='red_detector',
        executable='red_detector',
        output='screen'
    )
    
    red_obj_server = Node(
        package='red_detector',
        executable='red_obj_server',
        output='screen'
    )
    
    return LaunchDescription([
        red_detector,
        red_obj_server
    ])