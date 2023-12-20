from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


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

    global_localzation_service_arg = DeclareLaunchArgument(
        'global_localization_service',
        default_value='/reinitialize_global_localization',
        description='global_localization_service'
    )

    auto_localization_arg = DeclareLaunchArgument(
        'auto_localization',
        default_value='false',
        description='auto localization'
    )

    auto_localization = LaunchConfiguration('auto_localization')

    nav_http_server = Node(
        package='nav_http_server',
        executable='nav_http_server',
        output='screen',
        arguments=[
            '--map-frame', LaunchConfiguration('map_frame'),
            '--navigate-to-pose-action', LaunchConfiguration('navigate_to_pose_action'),
        ]
    )

    loc_http_server = Node(
        package='nav_http_server',
        executable='loc_http_server',
        output='screen',
        arguments=[
            '--robot-frame', LaunchConfiguration('robot_frame'),
            '--map-frame', LaunchConfiguration('map_frame'),
        ],
    )

    global_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('nav_http_server'), 'launch', 'global_localization.launch.py')]
        ),
        launch_arguments={
            'global_localization_service': LaunchConfiguration('global_localization_service')
        }.items(),
        condition=IfCondition(auto_localization)
    )

    return LaunchDescription([
        map_frame_arg,
        navigate_to_pose_action_arg,
        robot_frame_arg,
        global_localzation_service_arg,
        auto_localization_arg,
        nav_http_server,
        loc_http_server,
        global_localization_launch
    ])
