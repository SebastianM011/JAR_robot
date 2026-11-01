"""Localiza al robot real y reporta las cajas rojas en /victimas."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Sirve para el robot o para el bag (`ros2 bag play ... --clock`).

    No inicia drivers: /scan, /odom, la cámara y la TF odom -> base_footprint
    los provee el robot o el bag. No requiere Nav2.
    """
    def config(name):
        return LaunchConfiguration(name)

    def share(package, *parts):
        return PathJoinSubstitution([FindPackageShare(package), *parts])

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true al reproducir un bag con --clock'),
        DeclareLaunchArgument('map', default_value=share(
            'jar_bringup', 'maps', 'laberinto_vacio.yaml')),
        DeclareLaunchArgument('params_file', default_value=share(
            'jar_bringup', 'config', 'robot_real.yaml')),
        DeclareLaunchArgument('victimas', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=share(
            'jar_bringup', 'config', 'navegacion.rviz')),
    ]
    clock = {'use_sim_time': ParameterValue(config('use_sim_time'), value_type=bool)}
    params = [config('params_file'), clock]
    victimas = IfCondition(config('victimas'))
    return LaunchDescription(arguments + [
        Node(package='jar_localizacion', executable='publicar_mapa', name='publicar_mapa',
             output='screen', parameters=[clock, {'yaml_filename': config('map')}]),
        Node(package='jar_localizacion', executable='campo_verosimilitud',
             name='campo_verosimilitud', output='screen', parameters=params),
        Node(package='jar_localizacion', executable='localizador',
             name='localizador', output='screen', parameters=params),
        Node(package='jar_percepcion', executable='detector_scan',
             name='detector_scan_color', output='screen', parameters=params,
             condition=victimas),
        Node(package='jar_percepcion', executable='detector_obstaculos',
             name='detector_victimas', output='screen', parameters=params,
             condition=victimas,
             remappings=[('scan', 'scan_rojo'),
                         ('scan_no_mapeado', 'scan_victimas'),
                         ('objetos_no_mapeados', 'objetos_rojos'),
                         ('objetos_no_mapeados_markers', 'objetos_rojos_markers')]),
        Node(package='jar_mision', executable='juntar_victimas',
             name='juntar_victimas', output='screen', parameters=params,
             condition=victimas),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             condition=IfCondition(config('rviz')),
             arguments=['-d', config('rviz_config')], parameters=[clock],
             additional_env={'QT_QPA_PLATFORM': 'xcb'}),
    ])
