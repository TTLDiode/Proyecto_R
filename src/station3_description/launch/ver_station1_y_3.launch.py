"""
Launch de PRUEBA, solo para verificar visualmente ambos robots juntos
en Gazebo. No es parte de la linea oficial, es solo diagnostico.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_s3 = get_package_share_directory('station3_description')
    pkg_s1 = get_package_share_directory('station1_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_s3, 'worlds', 'station3_test.sdf')

    xacro_s3 = os.path.join(pkg_s3, 'urdf', 'station3.urdf.xacro')
    xacro_s1 = os.path.join(pkg_s1, 'urdf', 'station1.urdf.xacro')

    rsp_s3 = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='station3', output='screen',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', xacro_s3, ' control:=false']), value_type=str),
            'use_sim_time': True}])

    rsp_s1 = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', xacro_s1]), value_type=str),
            'use_sim_time': True}])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items())

    spawn_s3 = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', '/station3/robot_description',
                   '-name', 'station3', '-x', '0', '-y', '0', '-z', '0'])

    spawn_s1 = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'station1', '-x', '-0.91', '-y', '0', '-z', '0'])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_s3)),
        gz_sim, clock_bridge, rsp_s3, rsp_s1, spawn_s3, spawn_s1,
    ])
