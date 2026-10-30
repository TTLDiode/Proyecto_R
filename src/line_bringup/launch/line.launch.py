"""Lanza la linea completa.

Por ahora las tres estaciones se ejecutan como nodos de prueba, porque todavia
no existen sus controladores. A medida que cada estacion este lista, se retira
su identificador del argumento mock_stations.

También lanza el reponedor de pieza (tras cada fin de ciclo de la E3).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _setup(context, *args, **kwargs):
    poses_file = os.path.join(
        get_package_share_directory('line_bringup'), 'config', 'line_poses.yaml')

    mocks = [s.strip() for s in
             LaunchConfiguration('mock_stations').perform(context).split(',')
             if s.strip()]
    n_parts = int(LaunchConfiguration('n_parts').perform(context))
    done_timeout = float(LaunchConfiguration('done_timeout_s').perform(context))
    start_timeout = float(LaunchConfiguration('start_timeout_s').perform(context))
    reponer = LaunchConfiguration('reponer').perform(context).lower() in (
        '1', 'true', 'yes')

    nodes = []
    for station in mocks:
        nodes.append(Node(
            package='line_orchestrator',
            executable='station_mock',
            name=f'{station}_mock',
            output='screen',
            parameters=[{
                'station_id': station,
                'duration_s': 3.0,
                'assign_category': station == 'station2',
                'poses_file': poses_file,
            }],
        ))

    nodes.append(Node(
        package='line_orchestrator',
        executable='orchestrator',
        name='orchestrator',
        output='screen',
        parameters=[{
            'n_parts': n_parts,
            'poses_file': poses_file,
            'done_timeout_s': done_timeout,
            'start_timeout_s': start_timeout,
        }],
    ))

    if reponer:
        nodes.append(Node(
            package='line_orchestrator',
            executable='reponedor',
            name='reponedor_pieza',
            output='screen',
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mock_stations',
            default_value='station1,station2,station3',
            description='Estaciones mock, separadas por comas. Vacio = todas reales.'),
        DeclareLaunchArgument(
            'n_parts',
            default_value='3',
            description='Numero de piezas que procesa la corrida.'),
        DeclareLaunchArgument(
            'done_timeout_s',
            default_value='180.0',
            description='Timeout de pared esperando /line/stationN/done.'),
        DeclareLaunchArgument(
            'start_timeout_s',
            default_value='30.0',
            description='Timeout de pared esperando respuesta a /start.'),
        DeclareLaunchArgument(
            'reponer',
            default_value='true',
            description='Si true, lanza el reponedor automatico de pieza.'),
        OpaqueFunction(function=_setup),
    ])