"""Inicia Nav2 (localización, costmaps, planner, controller y BT navigator)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Reemplaza la navegación propia por los nodos estándar de Nav2.

    Con `localizacion:=false` no inicia map_server ni AMCL, para usar
    jar_localizacion u otro proveedor de /map y map -> odom.
    """
    def config(name):
        return LaunchConfiguration(name)

    def share(package, *parts):
        return PathJoinSubstitution([FindPackageShare(package), *parts])

    arguments = [
        DeclareLaunchArgument('simulacion', default_value='true'),
        DeclareLaunchArgument('localizacion', default_value='true',
                              description='Iniciar map_server y AMCL de Nav2'),
        DeclareLaunchArgument('use_sim_time', default_value=config('simulacion')),
        DeclareLaunchArgument('world', default_value='laberinto_simple_victimas.world'),
        DeclareLaunchArgument('map', default_value=share(
            'yahboom_rosmaster_gazebo', 'maps', 'laberinto_simple.yaml')),
        DeclareLaunchArgument('params_file', default_value=share(
            'jar_navegacion', 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=share(
            'nav2_bringup', 'rviz', 'nav2_default_view.rviz')),
        DeclareLaunchArgument('autostart', default_value='true'),
    ]
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share(
            'yahboom_rosmaster_bringup', 'launch', 'rosmaster_x3_sim.launch.py')),
        condition=IfCondition(config('simulacion')),
        launch_arguments={'world': config('world'), 'gui': config('gui'),
                          'use_sim_time': config('use_sim_time'), 'rviz': 'false'}.items(),
    )
    nav2_args = {'params_file': config('params_file'), 'use_sim_time': config('use_sim_time'),
                 'autostart': config('autostart')}
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('nav2_bringup', 'launch', 'localization_launch.py')),
        condition=IfCondition(config('localizacion')),
        launch_arguments={**nav2_args, 'map': config('map')}.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('nav2_bringup', 'launch', 'navigation_launch.py')),
        launch_arguments=nav2_args.items(),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(config('rviz')), arguments=['-d', config('rviz_config')],
        parameters=[{'use_sim_time': ParameterValue(config('use_sim_time'), value_type=bool)}])
    return LaunchDescription(arguments + [simulation, localization, navigation, rviz])
