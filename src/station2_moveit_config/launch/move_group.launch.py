"""Arranca move_group para la Estacion 2.

    ros2 launch station2_moveit_config move_group.launch.py
    ros2 launch station2_moveit_config move_group.launch.py rviz:=true

Se lanza despues de station2_description/launch/simulacion.launch.py. Adaptado
de station1_moveit_config. Diferencia: move_group corre en el espacio de
nombres station2, porque ahi viven los joint_states y los controladores de esta
estacion (/station2/joint_states, /station2/brazo_controller).

El URDF se expande con control:=false y acople:=false: MoveIt no necesita la
descripcion de hardware ni los complementos de Gazebo.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

ESPACIO = 'station2'


def _setup(context, *args, **kwargs):
    modelo = os.path.join(get_package_share_directory('station2_description'),
                          'urdf', 'station2.urdf.xacro')

    cfg = (
        MoveItConfigsBuilder('station2', package_name='station2_moveit_config')
        .robot_description(file_path=modelo,
                           mappings={'prefijo': 'station2/',
                                     'control': 'false',
                                     'camara': 'false',
                                     'acople': 'false'})
        .robot_description_semantic(file_path='config/station2.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    reloj = {'use_sim_time': True}

    nodos = [
        Node(package='moveit_ros_move_group', executable='move_group',
             namespace=ESPACIO, output='screen',
             parameters=[cfg.to_dict(), reloj]),
    ]

    rviz_cfg = os.path.join(
        get_package_share_directory('station2_moveit_config'),
        'config', 'moveit.rviz')
    nodos.append(
        Node(package='rviz2', executable='rviz2', output='screen',
             namespace=ESPACIO,
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', rviz_cfg],
             parameters=[cfg.robot_description,
                         cfg.robot_description_semantic,
                         cfg.robot_description_kinematics,
                         cfg.joint_limits,
                         cfg.planning_pipelines,
                         reloj]))
    return nodos


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abrir RViz con el panel de MoveIt.'),
        OpaqueFunction(function=_setup),
    ])
