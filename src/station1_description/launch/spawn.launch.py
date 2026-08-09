"""Abre el mundo de la linea y spawnea la Estacion 1 en su pose base.

    ros2 launch station1_description spawn.launch.py
    ros2 launch station1_description spawn.launch.py gui:=false

Cubre el requisito de la Entrega 1 de construir el mundo en Gazebo y spawnear
el robot. Todavia sin control de trayectoria: el modelo aparece y se sostiene,
las articulaciones no se comandan.

La pose base se toma de config/line_poses.yaml, de modo que si la disposicion
de la linea cambia, la estacion se coloca donde corresponde sin editar nada.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context, *args, **kwargs):
    descripcion_pkg = get_package_share_directory('station1_description')
    bringup_pkg = get_package_share_directory('line_bringup')

    modelo = os.path.join(descripcion_pkg, 'urdf', 'station1.urdf.xacro')
    mundo = os.path.join(bringup_pkg, 'worlds', 'line.sdf')
    poses = os.path.join(bringup_pkg, 'config', 'line_poses.yaml')

    with open(poses, 'r') as fh:
        base = yaml.safe_load(fh)['station_bases']['station1']

    descripcion = ParameterValue(
        Command(['xacro ', modelo, ' prefijo:=station1/']), value_type=str)

    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    cmd = ['gz', 'sim', '-r'] + ([] if gui else ['-s']) + [mundo]

    return [
        ExecuteProcess(cmd=cmd, output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': descripcion, 'use_sim_time': True}],
        ),

        # El simulador tarda en levantar sus servicios; sin esta espera la
        # creacion del modelo falla por no encontrarlos.
        TimerAction(period=4.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'station1',
                    '-x', str(base['x']),
                    '-y', str(base['y']),
                    '-z', str(base['z']),
                    '-Y', str(base['yaw']),
                ],
            ),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Abrir la interfaz grafica de Gazebo.'),
        OpaqueFunction(function=_setup),
    ])
