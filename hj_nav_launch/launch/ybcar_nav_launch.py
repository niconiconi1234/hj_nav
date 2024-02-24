from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription



def generate_launch_description():
    navigation_dwa_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch', 'navigation_dwa_launch.py')]
                )
            )
        ]
    )
    # navigation_dwa_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch', 'navigation_dwa_launch.py')]
    #     )
    # )

    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch', 'laser_bringup_launch.py')]
        )
    )

    # nav_server_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(get_package_share_directory('nav_http_server'), 'launch', 'start.launch.py')]
    #     )
    # )
    nav_server_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('nav_http_server'), 'launch', 'start.launch.py')]
                )
            )
        ]
    )

    return LaunchDescription([
        navigation_dwa_launch,
        laser_bringup_launch,
        nav_server_launch
    ])
