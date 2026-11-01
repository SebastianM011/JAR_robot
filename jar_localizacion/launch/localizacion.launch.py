"""Inicia el mapa y el filtro de partículas propio, sin Nav2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Publica /map, /likelihood_map y la transformada map -> odom."""
    def config(name):
        return LaunchConfiguration(name)

    def share(package, *parts):
        return PathJoinSubstitution([FindPackageShare(package), *parts])

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mapa', default_value='true',
                              description='Publicar /map; false si ya existe'),
        DeclareLaunchArgument('map', default_value=share(
            'yahboom_rosmaster_gazebo', 'maps', 'laberinto_simple.yaml')),
        DeclareLaunchArgument('params_file', default_value=share(
            'jar_localizacion', 'config', 'localizacion.yaml')),
    ]
    clock = {'use_sim_time': ParameterValue(config('use_sim_time'), value_type=bool)}
    params = [config('params_file'), clock]
    return LaunchDescription(arguments + [
        Node(package='jar_localizacion', executable='publicar_mapa', name='publicar_mapa',
             output='screen', condition=IfCondition(config('mapa')),
             parameters=[clock, {'yaml_filename': config('map')}]),
        Node(package='jar_localizacion', executable='campo_verosimilitud',
             name='campo_verosimilitud', output='screen', parameters=params),
        Node(package='jar_localizacion', executable='localizador',
             name='localizador', output='screen', parameters=params),
    ])
