"""Levanta la Estacion 2 (TRIADA) en Gazebo con sus controladores, la camara
de inspeccion y las piezas de prueba.

    ros2 launch station2_description simulacion.launch.py
    ros2 launch station2_description simulacion.launch.py gui:=false
    ros2 launch station2_description simulacion.launch.py mundo:=false piezas_prueba:=false

Con mundo:=true (por defecto) abre el mundo de la linea, line.sdf, y es la
forma de probar la estacion sola. Con mundo:=false se anade a un Gazebo que ya
esta abierto, por ejemplo el que levanta la Estacion 1; es lo que hace
simulacion_en_linea.launch.py.

Adaptado de station1_description/launch/simulacion.launch.py (orden de
arranque por eventos y desacople inicial) y de
station3_description/launch/simulacion_en_linea.launch.py (controller_manager
con espacio de nombres propio).

Lo que este paquete anade al mundo, sin tocar line.sdf, que es compartido:
  - soporte_traspaso_2_3: soporte del punto de entrega a la Estacion 3,
    igual que el del traspaso 1 -> 2 (cara superior en z = 0.042).
  - puesto_inspeccion: soporte igual en (0.500, 0.200), 200 mm a +y de la
    base. La camara va 70 mm por detras de la mordaza y no alcanza el
    traspaso 1 -> 2 (250 mm de la base, la lente llega a 230.6): la pieza se
    deja aqui, se fotografia y se vuelve a coger. Ver
    station2_control/nodo_estacion.py.
  - pieza_roja, pieza_verde y pieza_azul: piezas de prueba de 20 x 20 x 12 mm
    y 20 g, las mismas que mueve la linea, una por categoria (A, B y C).
"""
import math
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

# Colores de las piezas de prueba y la categoria que les corresponde. La
# misma tabla esta en station2_control/vision.py.
PIEZAS_PRUEBA = {
    'pieza_roja':  ((0.80, 0.15, 0.15), 'A'),
    'pieza_verde': ((0.15, 0.70, 0.20), 'B'),
    'pieza_azul':  ((0.15, 0.25, 0.85), 'C'),
}

# Donde esperan las piezas de prueba que no se estan usando: en el suelo,
# fuera del alcance del brazo y de las otras estaciones.
APARCAMIENTO = {
    'pieza_roja':  (0.40, -0.30, 0.006),
    'pieza_verde': (0.50, -0.30, 0.006),
    'pieza_azul':  (0.60, -0.30, 0.006),
}


def sdf_pieza(nombre, rgb):
    r, g, b = rgb
    return (
        f'<sdf version="1.9"><model name="{nombre}"><link name="cuerpo">'
        '<inertial><mass>0.0200</mass><inertia>'
        '<ixx>9.067e-07</ixx><iyy>9.067e-07</iyy><izz>1.333e-06</izz>'
        '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
        '<collision name="collision"><geometry><box>'
        '<size>0.020 0.020 0.012</size></box></geometry>'
        '<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>'
        '</surface></collision>'
        '<visual name="visual"><geometry><box>'
        '<size>0.020 0.020 0.012</size></box></geometry>'
        f'<material><ambient>{r} {g} {b} 1</ambient>'
        f'<diffuse>{r} {g} {b} 1</diffuse></material></visual>'
        '</link></model></sdf>')


# Puesto de inspeccion, en el frame world. Mismo soporte que los traspasos.
PUESTO_INSPECCION = (0.500, 0.200)


def sdf_soporte(nombre, lado=0.060):
    return (
        f'<sdf version="1.9"><model name="{nombre}"><static>true</static>'
        '<link name="superficie">'
        '<collision name="collision"><geometry><box>'
        f'<size>{lado} {lado} 0.042</size></box></geometry></collision>'
        '<visual name="visual"><geometry><box>'
        f'<size>{lado} {lado} 0.042</size></box></geometry>'
        '<material><ambient>0.35 0.40 0.45 1</ambient>'
        '<diffuse>0.35 0.40 0.45 1</diffuse></material></visual>'
        '</link></model></sdf>')


def _setup(context, *args, **kwargs):
    descripcion_pkg = get_package_share_directory('station2_description')
    bringup_pkg = get_package_share_directory('line_bringup')

    modelo = os.path.join(descripcion_pkg, 'urdf', 'station2.urdf.xacro')
    controladores = os.path.join(descripcion_pkg, 'config',
                                 'station2_controllers.yaml')
    mundo = os.path.join(bringup_pkg, 'worlds', 'line.sdf')
    with open(os.path.join(bringup_pkg, 'config', 'line_poses.yaml')) as fh:
        poses = yaml.safe_load(fh)
    base = poses['station_bases']['station2']
    traspaso_23 = poses['handoff_2_3']['position']

    abrir_mundo = LaunchConfiguration('mundo').perform(context) == 'true'
    con_prueba = LaunchConfiguration('piezas_prueba').perform(context) == 'true'
    con_soporte = LaunchConfiguration('soporte_2_3').perform(context) == 'true'
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    camara = LaunchConfiguration('camara').perform(context)

    piezas = ['pieza'] + (list(PIEZAS_PRUEBA) if con_prueba else [])

    descripcion = ParameterValue(
        Command(['xacro ', modelo,
                 ' prefijo:=station2/',
                 ' control:=true',
                 ' camara:=', camara,
                 ' base_x:=', str(base['x']), ' base_y:=', str(base['y']),
                 ' base_yaw:=', str(math.radians(base['yaw'])),
                 ' piezas:=', ','.join(piezas),
                 ' controladores:=', controladores]),
        value_type=str)

    acciones = []

    if abrir_mundo:
        # Ver station1_description: gz_ros2_control se busca en
        # GZ_SIM_SYSTEM_PLUGIN_PATH, que Jazzy no rellena.
        ruta = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
        previo = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        acciones.append(SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            ruta + (os.pathsep + previo if previo else '')))
        cmd = ['gz', 'sim', '-r'] + ([] if gui else ['-s']) + [mundo]
        acciones.append(ExecuteProcess(cmd=cmd, output='screen'))

    acciones.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='station2', output='screen',
        parameters=[{'robot_description': descripcion,
                     'use_sim_time': True}]))

    # Puentes ROS <-> Gazebo. El reloj solo si este launch abre el mundo:
    # con la Estacion 1 ya lo cruza su launch, y dos puentes lo duplicarian.
    puentes = []
    if abrir_mundo:
        puentes.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')
    for n in piezas:
        puentes += [f'/station2/{n}/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                    f'/station2/{n}/detach@std_msgs/msg/Empty]gz.msgs.Empty']
    if camara == 'true':
        puentes += [
            '/station2/camara/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/station2/camara/camera_info@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo']
    acciones.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=puentes, output='screen',
        parameters=[{'use_sim_time': True}]))

    def crear(nombre, x, y, z, yaw=0.0, cadena=None, topico=None):
        argumentos = ['-name', nombre, '-x', str(x), '-y', str(y),
                      '-z', str(z), '-Y', str(yaw)]
        argumentos = (['-string', cadena] if cadena else
                      ['-topic', topico]) + argumentos
        return Node(package='ros_gz_sim', executable='create',
                    output='screen', arguments=argumentos)

    if con_soporte:
        acciones.append(crear('soporte_traspaso_2_3', traspaso_23['x'],
                              traspaso_23['y'], 0.021,
                              cadena=sdf_soporte('soporte_traspaso_2_3')))
    acciones.append(crear('puesto_inspeccion', PUESTO_INSPECCION[0],
                          PUESTO_INSPECCION[1], 0.021,
                          cadena=sdf_soporte('puesto_inspeccion', 0.040)))
    if con_prueba:
        for n, (rgb, _) in PIEZAS_PRUEBA.items():
            x, y, z = APARCAMIENTO[n]
            acciones.append(crear(n, x, y, z, cadena=sdf_pieza(n, rgb)))

    # Se crea en el origen: la union fija al mundo del URDF ya lleva la pose
    # de la base (ver station2.urdf.xacro, anclaje).
    robot = crear('station2', 0.0, 0.0, 0.0,
                  topico='/station2/robot_description')
    acciones.append(robot)

    def spawner(nombre):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[nombre, '--controller-manager',
                               '/station2/controller_manager',
                               # Con la camara simulada el paso de fisica
                               # va mas lento que el tiempo real y la
                               # activacion por defecto (5 s) no alcanza.
                               '--switch-timeout', '30',
                               '--controller-manager-timeout', '60'],
                    output='screen')

    difusor = spawner('joint_state_broadcaster')
    brazo = spawner('brazo_controller')
    mordaza = spawner('mordaza_controller')

    # Un desacople por pieza: cada DetachableJoint nace acoplado.
    soltar = [ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '-w', '1',
             f'/station2/{n}/detach', 'std_msgs/msg/Empty', '{}'],
        output='screen') for n in piezas]

    acciones += [
        RegisterEventHandler(OnProcessExit(target_action=robot,
                                           on_exit=[difusor])),
        RegisterEventHandler(OnProcessExit(target_action=difusor,
                                           on_exit=[brazo])),
        RegisterEventHandler(OnProcessExit(target_action=brazo,
                                           on_exit=[mordaza])),
        RegisterEventHandler(OnProcessExit(target_action=mordaza,
                                           on_exit=soltar)),

        # Nodo lógico de la estación (habla con el orquestador)
        Node(
            package='station2_control',
            executable='nodo_estacion',
            name='nodo_estacion',
            namespace='station2',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ]
    return acciones

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir la interfaz grafica de Gazebo.'),
        DeclareLaunchArgument('mundo', default_value='true',
                              description='Abrir line.sdf. Con false se anade '
                                          'a un Gazebo ya abierto.'),
        DeclareLaunchArgument('piezas_prueba', default_value='true',
                              description='Crear las tres piezas de colores.'),
        DeclareLaunchArgument('soporte_2_3', default_value='true',
                              description='Crear el soporte del traspaso 2 -> 3.'),
        DeclareLaunchArgument('camara', default_value='true',
                              description='Simular la camara de inspeccion.'),
        OpaqueFunction(function=_setup),
    ])
