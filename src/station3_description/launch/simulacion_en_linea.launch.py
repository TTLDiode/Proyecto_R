"""Agrega la Estacion 3, con controladores, su pieza y el nodo logico,
a un Gazebo que ya esta abierto con el mundo de la linea.

Uso:
    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:<workspace>/install/station3_description/share
    ros2 launch station1_description simulacion.launch.py
    ros2 launch station3_description simulacion_en_linea.launch.py
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('station3_description')
    bringup = get_package_share_directory('line_bringup')

    xacro_path = os.path.join(pkg, 'urdf', 'station3.urdf.xacro')
    controllers = os.path.join(pkg, 'config', 'station3_controllers.yaml')

    with open(os.path.join(bringup, 'config', 'line_poses.yaml')) as f:
        poses = yaml.safe_load(f)
    base = poses['station_bases']['station3']
    hand = poses['handoff_2_3']['position']

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' control:=true',
                 ' controladores:=', controllers,
                 ' pieza:=pieza_e3']),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='station3', output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}])

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', '/station3/robot_description',
                   '-name', 'station3',
                   '-x', str(base['x']), '-y', str(base['y']),
                   '-z', str(base['z']),
                   '-Y', str(math.radians(base['yaw']))])

    pieza_sdf = (
        '<sdf version="1.9"><model name="pieza_e3">'
        f'<pose>{hand["x"]} {hand["y"]} 0.030 0 0 0</pose>'
        '<link name="cuerpo">'
        '<inertial><mass>0.0200</mass><inertia>'
        '<ixx>9.067e-07</ixx><iyy>9.067e-07</iyy><izz>1.333e-06</izz>'
        '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
        '<collision name="collision"><geometry><box>'
        '<size>0.020 0.020 0.012</size></box></geometry>'
        '<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>'
        '</surface></collision>'
        '<visual name="visual"><geometry><box>'
        '<size>0.020 0.020 0.012</size></box></geometry>'
        '<material><ambient>0.15 0.15 0.80 1</ambient>'
        '<diffuse>0.15 0.15 0.80 1</diffuse></material></visual>'
        '</link></model></sdf>')

    spawn_pieza = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-string', pieza_sdf,
                   '-name', 'pieza_e3',
                   '-x', str(hand['x']), '-y', str(hand['y']),
                   '-z', '0.030'])

    acople_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/station3/pieza/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/station3/pieza/detach@std_msgs/msg/Empty]gz.msgs.Empty',
        ],
        output='screen')

    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/station3/controller_manager'])

    arm_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/station3/controller_manager'])

    desacople_inicial = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/station3/pieza/detach',
             'std_msgs/msg/Empty', '{}'],
        output='screen')

    # Nodo lógico de la estación (habla con el orquestador)
    nodo_estacion = Node(
        package='station3_control',
        executable='nodo_estacion',
        name='nodo_estacion',
        namespace='station3',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        rsp,
        acople_bridge,
        spawn_pieza,
        spawn_robot,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster, on_exit=[arm_controller])),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_controller, on_exit=[desacople_inicial])),

        # Nodo de la estación
        nodo_estacion,
    ])
