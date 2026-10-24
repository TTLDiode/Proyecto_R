"""Visualiza la Estacion 2 (TRIADA) en RViz2 con control manual de las articulaciones.

    ros2 launch station2_description display.launch.py

Abre RViz2 con el modelo y una ventana de deslizadores para mover cada
articulacion. Sirve para comprobar el arbol de transformadas, los rangos de
movimiento y que la geometria del modelo es la que se espera.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    paquete = get_package_share_directory('station2_description')
    modelo = os.path.join(paquete, 'urdf', 'station2.urdf.xacro')
    config_rviz = os.path.join(paquete, 'rviz', 'station2.rviz')

    descripcion = ParameterValue(
        Command(['xacro ', modelo, ' prefijo:=', LaunchConfiguration('prefijo'),
                 ' control:=false acople:=false camara:=false']),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'prefijo', default_value='station2/',
            description='Prefijo de los frames de la estacion.'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': descripcion}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', config_rviz],
            output='screen',
        ),
    ])
