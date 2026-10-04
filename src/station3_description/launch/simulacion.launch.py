import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('station3_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_path = os.path.join(pkg_share, 'urdf', 'station3.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'station3_test.sdf')
    controllers_path = os.path.join(pkg_share, 'config', 'station3_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' control:=true',
                 ' controladores:=', controllers_path]),
        value_type=str)

    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_share))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='station3',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}])

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/station3/robot_description', '-name', 'station3'])

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/station3/controller_manager'])

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/station3/controller_manager'])

    # Puente para el acople cinematico de la pieza (ADR-002). El plugin
    # DetachableJoint escucha mensajes nativos de Gazebo (gz.msgs.Empty),
    # asi que se necesita puentear desde std_msgs/Empty de ROS2.
    acople_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/station3/pieza/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/station3/pieza/detach@std_msgs/msg/Empty]gz.msgs.Empty',
        ],
        output='screen')

    # El plugin DetachableJoint nace acoplado pese a lo que dice su propia
    # documentacion (hallazgo de Estacion 1). Se publica un desacople unico
    # al arrancar para que el estado inicial sea el esperado por el codigo.
    desacople_inicial = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',
                 '/station3/pieza/detach', 'std_msgs/msg/Empty', '{}'],
            output='screen')])

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        clock_bridge,
        acople_bridge,
        desacople_inicial,
        spawn_robot,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller])),
    ])
