from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    global_localzation_service_arg = DeclareLaunchArgument(
        'global_localization_service',
        default_value='/reinitialize_global_localization',
        description='global_localization_service'
    )

    global_localization_client = Node(
        package='nav_http_server',
        executable='global_localization_client',
        output='screen',
        arguments=[
                '--global-localization-service', LaunchConfiguration('global_localization_service'),
        ],
    )

    return LaunchDescription([
        global_localzation_service_arg,
        global_localization_client
    ])
