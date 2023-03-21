import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    max_braking_speed = LaunchConfiguration(
        'max_braking_speed', default='-10.0')
    max_throttle_speed = LaunchConfiguration(
        'max_throttle_speed', default='10.0')
    max_steering_rad = LaunchConfiguration(
        'max_steering_rad', default='2.0')
    wheelbase = LaunchConfiguration(
        'wheelbase', default='1.8')

    # Nodes
    joy = Node(package='joy',
               executable='joy_node',
               parameters=[{
                   'deadzone': 0.0
               }]
               )

    logi_g29 = Node(package='logi_g29',
                    executable='logi-g29',
                    parameters=[{
                        'max_braking_speed': max_braking_speed,
                        'max_throttle_speed': max_throttle_speed,
                        'max_steering_rad': max_steering_rad,
                        'wheelbase': wheelbase
                    }],
                    remappings=[
                        ('/ack_vel', '/logi/ack_vel'),
                    ],
                    )  # TODO change other launch files to all have the same wheel max angle, add to main launch file

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_rviz',
                              default_value='false',
                              description='Use simulation time if true'),

        DeclareLaunchArgument('max_braking_speed',
                              default_value='-10.0',
                              description=''),
        DeclareLaunchArgument('max_throttle_speed',
                              default_value='10.0',
                              description=''),
        DeclareLaunchArgument('max_steering_rad',
                              default_value='2.0',
                              description=''),
        DeclareLaunchArgument('wheelbase',
                              default_value='1.8',
                              description=''),

        # Node
        joy,
        logi_g29
    ])
