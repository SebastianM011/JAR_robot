"""Inicia los detectores de color y de obstáculos no mapeados."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Requiere /likelihood_map y la transformada map -> odom."""
    def config(name):
        return LaunchConfiguration(name)

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('jar_percepcion'), 'config', 'percepcion.yaml'])),
    ]
    params = [config('params_file'),
              {'use_sim_time': ParameterValue(config('use_sim_time'), value_type=bool)}]
    return LaunchDescription(arguments + [
        Node(package='jar_percepcion', executable='detector_color',
             name='detector_color', output='screen', parameters=params),
        Node(package='jar_percepcion', executable='detector_scan',
             name='detector_scan_color', output='screen', parameters=params),
        Node(package='jar_percepcion', executable='detector_obstaculos',
             name='detector_obstaculos', output='screen', parameters=params),
    ])
