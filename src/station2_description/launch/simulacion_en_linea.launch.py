"""Agrega la Estacion 2 a un Gazebo que ya esta abierto con el mundo de la
linea, por ejemplo el que levanta la Estacion 1.

    ros2 launch station1_description simulacion.launch.py
    ros2 launch station2_description simulacion_en_linea.launch.py

Es simulacion.launch.py con mundo:=false (no abre Gazebo ni repite el puente
de /clock) y sin las piezas de prueba: en la linea la pieza es "pieza", la
que deja la Estacion 1 en el traspaso 1 -> 2.

La camara necesita que el Gazebo abierto tenga un motor de render. El
sistema de sensores lo carga el propio modelo, de modo que no hay que tocar
line.sdf.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = os.path.join(get_package_share_directory('station2_description'),
                        'launch', 'simulacion.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('soporte_2_3', default_value='true'),
        DeclareLaunchArgument('camara', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base),
            launch_arguments={
                'mundo': 'false',
                'piezas_prueba': 'false',
                'soporte_2_3': LaunchConfiguration('soporte_2_3'),
                'camara': LaunchConfiguration('camara'),
            }.items()),
    ])
