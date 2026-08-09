"""Lanza la linea completa.

Por ahora las tres estaciones se ejecutan como nodos de prueba, porque todavia
no existen sus controladores. A medida que cada estacion este lista, se retira
su identificador del argumento mock_stations.
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
        }],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mock_stations',
            default_value='station1,station2,station3',
            description='Estaciones que se ejecutan como nodo de prueba, separadas por comas.'),
        DeclareLaunchArgument(
            'n_parts',
            default_value='3',
            description='Numero de piezas que procesa la corrida.'),
        OpaqueFunction(function=_setup),
    ])
