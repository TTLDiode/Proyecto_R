"""Levanta la Estacion 1 en Gazebo con sus controladores listos para recibir
trayectorias.

    ros2 launch station1_description simulacion.launch.py
    ros2 launch station1_description simulacion.launch.py gui:=false

Se diferencia de spawn.launch.py, de la Entrega 1, en que aquel solo colocaba
el modelo en el mundo. Aqui el URDF trae ademas las interfaces de
ros2_control, de modo que el complemento gz_ros2_control arranca el
controller_manager dentro del simulador y este lanzador solo tiene que activar
los controladores que ya estan declarados.

El orden importa y no es negociable: el modelo no existe hasta que el servicio
de creacion de Gazebo responde, y el controller_manager no existe hasta que el
modelo esta en el mundo. De ahi que los spawners cuelguen del evento de salida
del proceso de creacion y no de un temporizador: un temporizador funciona en
esta maquina y falla en otra mas lenta, que es exactamente la clase de fallo
que aparece el dia de la sustentacion.
"""
import os

import yaml
from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context, *args, **kwargs):
    descripcion_pkg = get_package_share_directory('station1_description')
    bringup_pkg = get_package_share_directory('line_bringup')

    modelo = os.path.join(descripcion_pkg, 'urdf', 'station1.urdf.xacro')
    controladores = os.path.join(descripcion_pkg, 'config',
                                 'station1_controllers.yaml')
    mundo = os.path.join(bringup_pkg, 'worlds', 'line.sdf')
    poses = os.path.join(bringup_pkg, 'config', 'line_poses.yaml')

    with open(poses, 'r') as fh:
        base = yaml.safe_load(fh)['station_bases']['station1']

    descripcion = ParameterValue(
        Command(['xacro ', modelo,
                 ' prefijo:=station1/',
                 ' control:=true',
                 ' acople:=', LaunchConfiguration('acople'),
                 ' controladores:=', controladores]),
        value_type=str)

    # El complemento de ros2_control lo carga Gazebo, no ROS, y lo busca en
    # GZ_SIM_SYSTEM_PLUGIN_PATH. Los paquetes de ROS 2 Jazzy instalan la
    # biblioteca en lib/ del prefijo pero no alimentan esa variable, de modo
    # que hay que anadirla aqui o el modelo aparece en el mundo sin
    # controller_manager y los spawners esperan para siempre.
    ruta_plugins = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    previo = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    ruta_plugins = ruta_plugins + (os.pathsep + previo if previo else '')

    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    cmd = ['gz', 'sim', '-r'] + ([] if gui else ['-s']) + [mundo]

    crear = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'station1',
                   '-x', str(base['x']), '-y', str(base['y']),
                   '-z', str(base['z']), '-Y', str(base['yaw'])],
    )

    def spawner(nombre):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[nombre, '--controller-manager',
                               '/controller_manager'],
                    output='screen')

    difusor = spawner('joint_state_broadcaster')
    brazo = spawner('brazo_controller')
    mordaza = spawner('mordaza_controller')

    # El complemento DetachableJoint acopla la pieza al arrancar, aunque su
    # documentacion diga que espera una peticion. Comprobado: sin enviar nada,
    # la pieza queda pegada a la mordaza desde el primer instante y el brazo la
    # arrastra por la bandeja; el contacto frena las articulaciones y el
    # controlador aborta las trayectorias.
    #
    # Este aviso deja el mundo en el estado correcto, con la pieza suelta en la
    # bandeja. Va aqui y no en el nodo de la estacion a proposito: asi cualquier
    # nodo que mueva el brazo encuentra el mundo coherente, incluso los que no
    # saben que existe una pieza.
    soltar_al_arrancar = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/station1/pieza/detach',
             'std_msgs/msg/Empty', '{}'],
        output='screen')

    return [
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', ruta_plugins),

        ExecuteProcess(cmd=cmd, output='screen'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': descripcion,
                          'use_sim_time': True}]),

        # Puente del reloj de simulacion. Sin el, todo lo que use use_sim_time
        # se queda esperando un /clock que nunca llega.
        # Puente del reloj y de los dos avisos de acople de la pieza. Los
        # tres cruzan la frontera entre ROS y Gazebo, que no comparten bus:
        # el reloj entra y los avisos salen.
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/station1/pieza/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                 '/station1/pieza/detach@std_msgs/msg/Empty]gz.msgs.Empty',
             ],
             output='screen',
             parameters=[{'use_sim_time': True}]),

        crear,
        RegisterEventHandler(OnProcessExit(target_action=crear,
                                           on_exit=[difusor])),
        RegisterEventHandler(OnProcessExit(target_action=difusor,
                                           on_exit=[brazo])),
        RegisterEventHandler(OnProcessExit(target_action=brazo,
                                           on_exit=[mordaza])),
        RegisterEventHandler(OnProcessExit(target_action=mordaza,
                                           on_exit=[soltar_al_arrancar])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir la interfaz grafica de Gazebo.'),
        DeclareLaunchArgument(
            'acople', default_value='true',
            description='Cargar el acople cinematico de la pieza. Con false '
                        'el brazo se mueve sin poder coger nada, lo que sirve '
                        'para aislar un problema de control de uno de agarre.'),
        OpaqueFunction(function=_setup),
    ])
