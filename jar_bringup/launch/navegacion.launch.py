"""Inicia la navegación propia y sus proveedores opcionales."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Permite reemplazar simulación y localización de forma independiente."""
    def config(name):
        return LaunchConfiguration(name)

    def share(package, *parts):
        return PathJoinSubstitution([FindPackageShare(package), *parts])

    arguments = [
        DeclareLaunchArgument('simulacion', default_value='true'),
        DeclareLaunchArgument('localizacion', default_value='true',
                              description='Iniciar map_server y AMCL provisionales'),
        DeclareLaunchArgument('localizador', default_value='amcl', choices=['amcl', 'jar'],
                              description='AMCL de Nav2 o el filtro de jar_localizacion'),
        DeclareLaunchArgument('exploracion', default_value='true'),
        DeclareLaunchArgument('percepcion', default_value='true'),
        DeclareLaunchArgument('victimas', default_value='true'),
        DeclareLaunchArgument('cantidad_victimas', default_value='2',
                              description='Reportes que acepta la prueba: cajas rojas'),
        DeclareLaunchArgument('porcentaje_objetivo', default_value='98.0',
                              description='Porcentaje de celdas libres cubiertas para terminar'),
        DeclareLaunchArgument('confirmar_pose_automatica', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value=config('simulacion')),
        DeclareLaunchArgument('verificar_movimiento_simulado',
                              default_value=config('simulacion')),
        DeclareLaunchArgument('world', default_value='laberinto_simple.world'),
        DeclareLaunchArgument('map', default_value=share(
            'yahboom_rosmaster_gazebo', 'maps', 'laberinto_simple.yaml')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir RViz con la configuración de JAR'),
        DeclareLaunchArgument('rviz_config', default_value=share(
            'jar_bringup', 'config', 'navegacion.rviz'),
            description='Archivo .rviz que se abre junto al simulador'),
        DeclareLaunchArgument('rviz_software', default_value='false',
                              choices=['true', 'false'],
                              description='Probar renderizado por CPU para RViz'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('params_file', default_value=share(
            'jar_bringup', 'config', 'navegacion.yaml')),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
    ]
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share(
            'yahboom_rosmaster_bringup', 'launch', 'rosmaster_x3_sim.launch.py')),
        condition=IfCondition(config('simulacion')),
        launch_arguments={
            **{name: config(name) for name in (
                'world', 'gui', 'use_sim_time', 'spawn_x', 'spawn_y', 'spawn_yaw')},
            'rviz': 'false',
        }.items(),
    )
    clock = {'use_sim_time': ParameterValue(config('use_sim_time'), value_type=bool)}

    def localizador_es(nombre):
        return IfCondition(PythonExpression([
            "'", config('localizacion'), "' == 'true' and '",
            config('localizador'), "' == '", nombre, "'"]))

    localization_nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', condition=localizador_es('amcl'),
             parameters=[config('params_file'), clock, {'yaml_filename': config('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             condition=localizador_es('amcl'),
             parameters=[config('params_file'), clock]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localizacion', output='screen',
             condition=localizador_es('amcl'),
             parameters=[clock, {'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        # El localizador propio no necesita Nav2: el mapa lo publica jar_localizacion.
        Node(package='jar_localizacion', executable='publicar_mapa', name='publicar_mapa',
             output='screen', condition=localizador_es('jar'),
             parameters=[clock, {'yaml_filename': config('map')}]),
        Node(package='jar_localizacion', executable='campo_verosimilitud',
             name='campo_verosimilitud', output='screen',
             condition=IfCondition(PythonExpression([
                 "('", config('localizacion'), "' == 'true' and '",
                 config('localizador'), "' == 'jar') or '",
                 config('percepcion'), "' == 'true' or '", config('victimas'), "' == 'true'"])),
             parameters=[config('params_file'), clock]),
        Node(package='jar_localizacion', executable='localizador', name='localizador',
             output='screen', condition=localizador_es('jar'),
             parameters=[config('params_file'), clock]),
    ]
    # Cada include conserva sus parámetros en un ámbito propio.
    perception = GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('jar_percepcion', 'launch', 'percepcion.launch.py')),
        condition=IfCondition(config('percepcion')),
        launch_arguments={
            'use_sim_time': config('use_sim_time'),
            'params_file': share('jar_percepcion', 'config', 'percepcion.yaml'),
        }.items())])
    mission = GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('jar_mision', 'launch', 'victimas.launch.py')),
        condition=IfCondition(config('victimas')),
        launch_arguments={
            'use_sim_time': config('use_sim_time'),
            'params_file': share('jar_mision', 'config', 'mision.yaml'),
            'cantidad_victimas': config('cantidad_victimas'),
            'iniciar_detector_scan': PythonExpression([
                "'false' if '", config('percepcion'), "' == 'true' else 'true'"]),
        }.items())])
    coverage = Node(
        package='jar_exploracion', executable='grilla_cobertura',
        name='grilla_cobertura', output='screen', parameters=[config('params_file'), clock])
    explorer = Node(
        package='jar_exploracion', executable='explorador', name='explorador',
        output='screen', condition=IfCondition(config('exploracion')),
        parameters=[config('params_file'), clock, {
            'porcentaje_objetivo': ParameterValue(config('porcentaje_objetivo'), value_type=float),
            'confirmar_pose_automatica': ParameterValue(
                    config('confirmar_pose_automatica'), value_type=bool),
            'verificar_movimiento_simulado': ParameterValue(
                config('verificar_movimiento_simulado'), value_type=bool)}],
        remappings=[('cmd_vel', config('cmd_vel_topic'))])
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(config('rviz')),
        arguments=['-d', config('rviz_config')], parameters=[clock],
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
            'LIBGL_ALWAYS_SOFTWARE': PythonExpression([
                "'1' if '", config('rviz_software'), "' == 'true' else '0'"]),
        })
    visualizador = Node(
        package='jar_bringup', executable='visualizador_grillas',
        output='screen', condition=IfCondition(config('rviz')), parameters=[clock])
    # Resolver nuestro RViz antes de que el include establezca rviz=false.
    # El simulador necesita conservar sus configuraciones para OnProcessExit.
    return LaunchDescription(
        arguments + [visualizador, rviz, simulation] + localization_nodes
        + [perception, mission, coverage, explorer])
