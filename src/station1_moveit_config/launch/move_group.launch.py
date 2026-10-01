"""Arranca move_group para la Estacion 1.

    ros2 launch station1_moveit_config move_group.launch.py
    ros2 launch station1_moveit_config move_group.launch.py rviz:=true

Se lanza despues de station1_description/launch/simulacion.launch.py, que es
quien levanta Gazebo y los controladores. Aqui solo aparece la capa de
planificacion: move_group lee el mismo URDF que el simulador, le anade la
descripcion semantica del SRDF y manda las trayectorias a los controladores ya
activos.

El URDF se expande con control:=false a proposito. MoveIt no necesita las
etiquetas de ros2_control ni el complemento de Gazebo, que ademas ya estan
cargados en el simulador; dejarlas fuera evita que move_group intente
interpretar una descripcion de hardware que no le corresponde.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _setup(context, *args, **kwargs):
    modelo = os.path.join(get_package_share_directory('station1_description'),
                          'urdf', 'station1.urdf.xacro')

    cfg = (
        MoveItConfigsBuilder('station1', package_name='station1_moveit_config')
        .robot_description(file_path=modelo,
                           mappings={'prefijo': 'station1/',
                                     'control': 'false'})
        .robot_description_semantic(file_path='config/station1.srdf')
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
             output='screen',
             parameters=[cfg.to_dict(), reloj]),
    ]

    rviz_cfg = os.path.join(
        get_package_share_directory('station1_moveit_config'),
        'config', 'moveit.rviz')
    nodos.append(
        Node(package='rviz2', executable='rviz2', output='screen',
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
