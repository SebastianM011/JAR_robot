"""Inicia el evasor reactivo, opcionalmente junto al simulador."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Publica en cmd_vel: no combinar con el explorador ni con Nav2."""
    def config(name):
        return LaunchConfiguration(name)

    def share(package, *parts):
        return PathJoinSubstitution([FindPackageShare(package), *parts])

    arguments = [
        DeclareLaunchArgument('simulacion', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value=config('simulacion')),
        DeclareLaunchArgument('world', default_value='laberinto_simple.world'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=share(
            'jar_navegacion', 'config', 'evasor.yaml')),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
    ]
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share(
            'yahboom_rosmaster_bringup', 'launch', 'rosmaster_x3_sim.launch.py')),
        condition=IfCondition(config('simulacion')),
        launch_arguments={'world': config('world'), 'gui': config('gui'),
                          'use_sim_time': config('use_sim_time'), 'rviz': 'false'}.items(),
    )
    evasor = Node(
        package='jar_navegacion', executable='evasor', name='evasor', output='screen',
        parameters=[config('params_file'), {
            'use_sim_time': ParameterValue(config('use_sim_time'), value_type=bool)}],
        remappings=[('cmd_vel', config('cmd_vel_topic'))])
    return LaunchDescription(arguments + [simulation, evasor])
