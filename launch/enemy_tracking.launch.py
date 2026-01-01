from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_topic', default_value='armor_solver/target'),
        DeclareLaunchArgument('costmap_topic', default_value='/global_costmap/costmap'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('attack_radius', default_value='3.0'),
        DeclareLaunchArgument('num_sectors', default_value='36'),
        DeclareLaunchArgument('cost_threshold', default_value='50'),
        DeclareLaunchArgument('goal_update_interval', default_value='1.0'),
        DeclareLaunchArgument('enable_navigation', default_value='true'),
        DeclareLaunchArgument('tf_tolerance', default_value='0.5'),

        Node(
            package='enemy_tracking',
            executable='enemy_tracking_node',
            name='enemy_tracking_node',
            parameters=[{
                'target_topic': LaunchConfiguration('target_topic'),
                'costmap_topic': LaunchConfiguration('costmap_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'attack_radius': LaunchConfiguration('attack_radius'),
                'num_sectors': LaunchConfiguration('num_sectors'),
                'cost_threshold': LaunchConfiguration('cost_threshold'),
                'goal_update_interval': LaunchConfiguration('goal_update_interval'),
                'enable_navigation': LaunchConfiguration('enable_navigation'),
                'tf_tolerance': LaunchConfiguration('tf_tolerance'),
            }],
            output='screen',
        ),
    ])
