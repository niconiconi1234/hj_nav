from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='map frame'
    )

    navigate_to_pose_action_arg = DeclareLaunchArgument(
        'navigate_to_pose_action',
        default_value='/navigate_to_pose',
        description='navigate_to_pose_action'
    )

    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame',
        default_value='base_link',
        description='robot frame'
    )

    nav_http_server = Node(
        package='nav_http_server',
        executable='nav_http_server',
        name='nav_http_server',
        output='screen',
        arguments=[
            '--map-frame', LaunchConfiguration('map_frame'),
            '--navigate-to-pose-action', LaunchConfiguration('navigate_to_pose_action'),
        ]
    )

    loc_http_server = Node(
        package='nav_http_server',
        executable='loc_http_server',
        name='loc_http_server',
        output='screen',
        arguments=[
            '--robot-frame', LaunchConfiguration('robot_frame'),
            '--map-frame', LaunchConfiguration('map_frame'),
        ],
    )

    return LaunchDescription([
        map_frame_arg,
        navigate_to_pose_action_arg,
        robot_frame_arg,
        nav_http_server,
        loc_http_server
    ])
