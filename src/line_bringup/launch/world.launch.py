"""Abre el mundo de la linea en Gazebo.

    ros2 launch line_bringup world.launch.py                 con interfaz grafica
    ros2 launch line_bringup world.launch.py gui:=false      solo el servidor
    ros2 launch line_bringup world.launch.py paused:=true    arranca en pausa

El servidor sin interfaz es util para medir tiempos de ciclo y para las pruebas
automaticas, porque no gasta GPU en dibujar.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    world = os.path.join(
        get_package_share_directory('line_bringup'), 'worlds', 'line.sdf')

    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'

    cmd = ['gz', 'sim']
    if not gui:
        cmd.append('-s')          # solo servidor, sin dibujar
    if not paused:
        cmd.append('-r')          # empieza a simular sin esperar
    cmd.append(world)

    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Abrir la interfaz grafica de Gazebo.'),
        DeclareLaunchArgument(
            'paused', default_value='false',
            description='Arrancar la simulacion en pausa.'),
        OpaqueFunction(function=_setup),
    ])
