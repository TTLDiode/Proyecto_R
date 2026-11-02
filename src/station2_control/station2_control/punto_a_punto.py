"""Nodo de control punto a punto de la Estacion 2, con cinematica propia.

    ros2 run station2_control punto_a_punto -- 0.250 0.000 0.060
    ros2 run station2_control punto_a_punto --pose traspaso_12
    ros2 run station2_control punto_a_punto --ciclo

Adaptado de station1_control/punto_a_punto.py. Es la via "codigo propio" de
la Entrega 2, la que se contrasta con MoveIt:

    punto cartesiano (world)
        -> cinematica.a_base + cinematica.inversa   forma cerrada
        -> trayectoria.muestrear                    perfil quintico
        -> FollowJointTrajectory                    al brazo_controller

Al terminar lee el arbol TF y contrasta la pose alcanzada con la pedida: es
la medida que cierra el lazo contra el simulador y no contra la propia
cinematica.

Diferencias con la Estacion 1: los puntos llegan en el frame world y la base
esta en x = 0.5, y el controller_manager vive en /station2.
"""
import argparse
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from station2_control import cinematica as cin
from station2_control import trayectoria as tra

# Puntos con nombre, en el frame world. Los traspasos son los de
# line_bringup/config/line_poses.yaml; el puesto de inspeccion es propio de
# esta estacion (ver station2_description/launch/simulacion.launch.py).
PUNTOS = {
    'traspaso_12': (0.250, 0.000, 0.060),
    'traspaso_23': (0.720, 0.000, 0.060),
    'inspeccion':  (0.500, 0.200, 0.060),
}

# Mordaza: lo que se cierra cada dedo desde la apertura maxima de 60 mm.
DEDO_ABIERTO = 0.0
DEDO_CERRADO = 0.0195          # 21 mm entre caras, pieza de 20 mm
APERTURA_MAX = 0.060
LADO_PIEZA = 0.020
HOLGURA = 0.001


def cierre_para(giro):
    """Cierre de cada dedo para una pieza cuadrada girada 'giro' (rad)
    respecto a la pinza.

    El SCARA no tiene muneca: la pinza queda girada theta1 + theta2 y la pieza
    llega con el giro que le dio la Estacion 1 (0 en line_poses.yaml). Girada,
    la pieza mide lado * (|cos| + |sen|) en la direccion de cierre (28 mm a 36
    grados), y los dedos se detienen ahi, en las esquinas, sin atravesarla.
    """
    ancho = LADO_PIEZA * (abs(math.cos(giro)) + abs(math.sin(giro)))
    return (APERTURA_MAX - ancho - HOLGURA) / 2.0


class ControlPuntoAPunto(Node):

    def __init__(self, nombre='station2_punto_a_punto'):
        super().__init__(
            nombre,
            parameter_overrides=[Parameter('use_sim_time',
                                           Parameter.Type.BOOL, True)])
        self._estado = None
        self.create_subscription(JointState, '/station2/joint_states',
                                 self._al_llegar_estado, 10)
        self._accion = ActionClient(
            self, FollowJointTrajectory,
            '/station2/brazo_controller/follow_joint_trajectory')
        self._mordaza = None
        # Ver station1_control/punto_a_punto.py: dentro de nodo_estacion el
        # nodo lo gira un ejecutor y aqui solo se espera.
        self._girado_fuera = False
        self._tf = Buffer()
        self._escucha = TransformListener(self._tf, self)

    # ------------------------------------------------------------------
    def _girar(self, segundos):
        if self._girado_fuera:
            time.sleep(segundos)
        else:
            rclpy.spin_once(self, timeout_sec=segundos)

    def _esperar(self, futuro, plazo=60.0):
        if self._girado_fuera:
            limite = time.time() + plazo
            while not futuro.done() and time.time() < limite:
                time.sleep(0.005)
        else:
            rclpy.spin_until_future_complete(self, futuro, timeout_sec=plazo)
        return futuro.result() if futuro.done() else None

    def _al_llegar_estado(self, msg):
        try:
            self._estado = [msg.position[msg.name.index(j)]
                            for j in cin.ARTICULACIONES]
        except ValueError:
            pass

    def esperar_estado(self, plazo=15.0):
        limite = time.time() + plazo
        while self._estado is None and time.time() < limite:
            self._girar(0.1)
        if self._estado is None:
            raise RuntimeError('no llegan estados articulares; '
                               'esta corriendo la simulacion?')
        return list(self._estado)

    def pose_marco(self, marco, plazo=5.0):
        """Posicion de un marco en el mundo, leida del arbol TF."""
        limite = time.time() + plazo
        while time.time() < limite:
            self._girar(0.05)
            try:
                t = self._tf.lookup_transform('world', marco,
                                              rclpy.time.Time())
                v = t.transform.translation
                return (v.x, v.y, v.z)
            except Exception:
                continue
        raise RuntimeError('no hay transformada world -> %s' % marco)

    def pose_herramienta(self, plazo=5.0):
        return self.pose_marco('station2/tool0', plazo)

    # ------------------------------------------------------------------
    def mover_a_articular(self, q_objetivo, escala_vel=1.0, escala_ace=1.0):
        """Ejecuta el perfil propio hasta q_objetivo. Devuelve (ok, T, t_real)."""
        q0 = self.esperar_estado()
        T = tra.duracion(q0, q_objetivo, escala_vel=escala_vel,
                         escala_ace=escala_ace)
        msg = JointTrajectory()
        msg.joint_names = list(cin.ARTICULACIONES)
        for t, q, qd in tra.muestrear(q0, q_objetivo, T):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in q]
            p.velocities = [float(v) for v in qd]
            p.time_from_start = Duration(sec=int(t),
                                         nanosec=int((t % 1.0) * 1e9))
            msg.points.append(p)

        if not self._accion.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('el brazo_controller no ofrece '
                               'follow_joint_trajectory')
        meta = FollowJointTrajectory.Goal()
        meta.trajectory = msg
        t0 = time.time()
        manejador = self._esperar(self._accion.send_goal_async(meta), 20.0)
        if manejador is None or not manejador.accepted:
            return False, T, 0.0
        resultado = self._esperar(manejador.get_result_async(), 4*T + 30.0)
        t_real = time.time() - t0
        ok = resultado is not None and resultado.result.error_code == 0
        return ok, T, t_real

    def mover_mordaza(self, cierre, duracion=0.6):
        """Mueve los dos dedos a la vez; el servo los arrastra por el piñon."""
        if self._mordaza is None:
            self._mordaza = ActionClient(
                self, FollowJointTrajectory,
                '/station2/mordaza_controller/follow_joint_trajectory')
        if not self._mordaza.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('el mordaza_controller no responde')
        msg = JointTrajectory()
        msg.joint_names = ['station2/dedo_1_joint', 'station2/dedo_2_joint']
        p = JointTrajectoryPoint()
        p.positions = [float(cierre), float(cierre)]
        p.velocities = [0.0, 0.0]
        p.time_from_start = Duration(sec=int(duracion),
                                     nanosec=int((duracion % 1.0) * 1e9))
        msg.points.append(p)
        meta = FollowJointTrajectory.Goal()
        meta.trajectory = msg
        manejador = self._esperar(self._mordaza.send_goal_async(meta), 15.0)
        if manejador is None or not manejador.accepted:
            return False
        res = self._esperar(manejador.get_result_async(), duracion + 15.0)
        return res is not None and res.result.error_code == 0

    def abrir_mordaza(self):
        return self.mover_mordaza(DEDO_ABIERTO)

    def cerrar_mordaza(self, cierre=DEDO_CERRADO):
        return self.mover_mordaza(cierre)

    def mover_a_punto(self, punto, rama='auto', **kw):
        """Lleva tool0 a un punto del frame world y mide el error real."""
        q, ok, motivo = cin.inversa(cin.a_base(punto), rama=rama)
        if not ok:
            self.get_logger().error(f'punto {punto} inalcanzable: {motivo}')
            return None
        self.get_logger().info(
            'objetivo %s  ->  theta1 %.2f  theta2 %.2f  d3 %.1f mm  (codo %s)'
            % (punto, math.degrees(q[0]), math.degrees(q[1]), q[2]*1000,
               'arriba' if q[1] > 0 else 'abajo'))
        exito, T, t_real = self.mover_a_articular(q, **kw)
        alcanzado = self.pose_herramienta()
        error = math.dist(alcanzado, punto)
        self.get_logger().info(
            'ejecutado en %.2f s (perfil %.2f s), error final %.3f mm, %s'
            % (t_real, T, error*1000, 'OK' if exito else 'FALLO'))
        return {'q': q, 'T': T, 't_real': t_real, 'error': error, 'ok': exito,
                'alcanzado': alcanzado}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('coordenadas', nargs='*', type=float,
                    help='x y z en metros, en el frame del mundo')
    ap.add_argument('--pose', choices=sorted(PUNTOS),
                    help='punto con nombre')
    ap.add_argument('--ciclo', action='store_true',
                    help='traspaso 1-2 -> inspeccion -> traspaso 2-3')
    ap.add_argument('--rama', choices=('auto', 'arriba', 'abajo'),
                    default='auto')
    args = ap.parse_args(argv if argv is not None else
                         rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    nodo = ControlPuntoAPunto()
    try:
        if args.ciclo:
            t0 = time.time()
            for nombre in ('traspaso_12', 'inspeccion', 'traspaso_23'):
                nodo.mover_a_punto(PUNTOS[nombre], rama=args.rama)
            nodo.get_logger().info('recorrido completo en %.1f s'
                                   % (time.time() - t0))
        elif args.pose:
            nodo.mover_a_punto(PUNTOS[args.pose], rama=args.rama)
        elif len(args.coordenadas) == 3:
            nodo.mover_a_punto(tuple(args.coordenadas), rama=args.rama)
        else:
            ap.error('indica tres coordenadas, --pose o --ciclo')
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
