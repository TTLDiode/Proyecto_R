"""Visualiza la Estacion 2 (TRIADA) en RViz2 con control manual de las articulaciones.

    ros2 launch station2_description display.launch.py

Abre RViz2 con el modelo y una ventana de deslizadores para mover cada
articulacion. Sirve para comprobar el arbol de transformadas, los rangos de
movimiento y que la geometria del modelo es la que se espera.

La base queda en el origen de world (base_x:=0), en el centro de la rejilla;
en Gazebo va en su sitio de la linea, x = 0.5 (line_poses.yaml). Arranca en
la pose de reposo, theta1 = 90 grados (brazo hacia +y, la zona de trabajo),
como en Gazebo; con theta1 = 0 el brazo queda sobre la PCB (+x), que es la
pose en que esta dibujado el CAD.
"""
import math
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
                 ' control:=false acople:=false camara:=false',
                 ' base_x:=0.0 mallas:=', LaunchConfiguration('mallas')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'prefijo', default_value='station2/',
            description='Prefijo de los frames de la estacion.'),
        DeclareLaunchArgument(
            'mallas', default_value='true',
            description='Visual con las mallas del CAD (false: solidos simples).'),

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
            # Pose de reposo al abrir; el boton Center la vuelve a cero.
            parameters=[{'zeros': {'station2/joint_1': math.pi / 2}}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', config_rviz],
            output='screen',
        ),
    ])
