from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gimbal_frame', default_value='gimbal'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('enemy_distance_topic', default_value='/enemy_distance'),
        DeclareLaunchArgument('min_tracking_distance', default_value='0.5'),
        DeclareLaunchArgument('max_tracking_distance', default_value='10.0'),
        DeclareLaunchArgument('goal_update_interval', default_value='1.0'),
        DeclareLaunchArgument('goal_offset', default_value='0.3'),

        Node(
            package='enemy_tracking',
            executable='enemy_tracking_node',
            name='enemy_tracking_node',
            parameters=[{
                'gimbal_frame': LaunchConfiguration('gimbal_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
                'enemy_distance_topic': LaunchConfiguration('enemy_distance_topic'),
                'min_tracking_distance': LaunchConfiguration('min_tracking_distance'),
                'max_tracking_distance': LaunchConfiguration('max_tracking_distance'),
                'goal_update_interval': LaunchConfiguration('goal_update_interval'),
                'goal_offset': LaunchConfiguration('goal_offset'),
            }],
            output='screen',
        ),
    ])