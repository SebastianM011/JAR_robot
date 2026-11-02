"""Ubica las cajas rojas en el mapa y las reporta en /victimas."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Encadena /scan_rojo -> detector de obstáculos -> /objetos_rojos -> /victimas.

    Requiere /likelihood_map y la transformada map -> odom (por ejemplo,
    con jar_localizacion).
    """
    def config(name):
        return LaunchConfiguration(name)

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cantidad_victimas', default_value='2',
                              description='Reportes que acepta la prueba: cajas rojas'),
        DeclareLaunchArgument('iniciar_detector_scan', default_value='true',
                              description='false si percepción ya publica /scan_rojo'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('jar_mision'), 'config', 'mision.yaml'])),
    ]
    params = [config('params_file'),
              {'use_sim_time': ParameterValue(config('use_sim_time'), value_type=bool)}]
    return LaunchDescription(arguments + [
        Node(package='jar_percepcion', executable='detector_scan',
             name='detector_scan_color', output='screen', parameters=params,
             condition=IfCondition(config('iniciar_detector_scan'))),
        Node(package='jar_percepcion', executable='detector_obstaculos',
             name='detector_victimas', output='screen', parameters=params,
             remappings=[('scan', 'scan_rojo'),
                         ('scan_no_mapeado', 'scan_victimas'),
                         ('objetos_no_mapeados', 'objetos_rojos'),
                         ('objetos_no_mapeados_markers', 'objetos_rojos_markers')]),
        Node(package='jar_mision', executable='juntar_victimas',
             name='juntar_victimas', output='screen', parameters=params + [{
                 'victimas': ParameterValue(config('cantidad_victimas'), value_type=int)}]),
    ])
