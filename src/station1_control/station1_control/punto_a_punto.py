"""Nodo de control punto a punto de la Estacion 1, con cinematica propia.

    ros2 run station1_control punto_a_punto -- -0.180 0.100 0.050
    ros2 run station1_control punto_a_punto --pose bandeja
    ros2 run station1_control punto_a_punto --ciclo

Es la via "codigo propio" de la Entrega 2, la que se contrasta con MoveIt:

    punto cartesiano
        -> cinematica.inversa        forma cerrada, sin biblioteca
        -> trayectoria.muestrear     perfil quintico sincronizado
        -> FollowJointTrajectory     al brazo_controller de ros2_control

No hay planificador de por medio: el movimiento es una interpolacion
articular entre la pose actual y la pose objetivo. Para esta estacion es
suficiente y es lo correcto, porque su volumen de trabajo esta despejado, y es
justamente la diferencia que la tabla comparativa tiene que medir frente a
MoveIt, que ademas comprueba colisiones.

Al terminar, el nodo lee el arbol TF y contrasta la pose alcanzada con la
pedida. Esa comprobacion no es decorativa: es la unica que cierra el lazo
contra el simulador y no contra la propia cinematica, que siempre se da la
razon a si misma.
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

from station1_control import cinematica as cin
from station1_control import trayectoria as tra

# Puntos con nombre, tomados de line_bringup/config/line_poses.yaml
PUNTOS = {
    'bandeja':   (-0.180, 0.100, 0.050),
    'esquina1':  (-0.230, 0.050, 0.050),
    'esquina2':  (-0.230, 0.150, 0.050),
    'esquina3':  (-0.130, 0.050, 0.050),
    'esquina4':  (-0.130, 0.150, 0.050),
    'traspaso':  ( 0.250, 0.000, 0.060),
}


class ControlPuntoAPunto(Node):

    def __init__(self):
        # El reloj lo manda Gazebo. Se pasa como sustitucion y no con
        # declare_parameter porque rclpy ya declara use_sim_time por su cuenta.
        super().__init__(
            'station1_punto_a_punto',
            parameter_overrides=[Parameter('use_sim_time',
                                           Parameter.Type.BOOL, True)])

        self._estado = None
        self.create_subscription(JointState, '/joint_states', self._al_llegar_estado, 10)

        self._accion = ActionClient(
            self, FollowJointTrajectory,
            '/brazo_controller/follow_joint_trajectory')

        self._mordaza = None        # se crea la primera vez que se usa

        # Lo pone a True quien meta este nodo en un ejecutor propio; ver
        # _girar y _esperar.
        self._girado_fuera = False

        self._tf = Buffer()
        self._escucha = TransformListener(self._tf, self)

    # ------------------------------------------------------------------
    # Espera de eventos. Hay dos situaciones y no dan lo mismo.
    #
    # Ejecutado como script, este nodo es el unico y se gira a si mismo.
    # Dentro de nodo_estacion, en cambio, el nodo ya lo gira un ejecutor y el
    # ciclo corre en un hilo aparte: volver a girarlo desde ese hilo se lo
    # quita al ejecutor, y entonces ni se entregan las respuestas de servicio
    # ni avanzan los futuros. Se manifestaba como una llamada a
    # /line/station1/start que no respondia nunca.
    #
    # Cual de las dos situaciones es se declara, no se adivina. La propiedad
    # executor del nodo no sirve para distinguirlas: rclpy.spin_once anade el
    # nodo al ejecutor global y lo quita en cada llamada, de modo que el valor
    # depende de cuando se mire.
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
            pass        # todavia no estan las tres

    def esperar_estado(self, plazo=15.0):
        limite = time.time() + plazo
        while self._estado is None and time.time() < limite:
            self._girar(0.1)
        if self._estado is None:
            raise RuntimeError('no llegan estados articulares; '
                               'esta corriendo la simulacion?')
        return list(self._estado)

    def pose_herramienta(self, plazo=5.0):
        """Pose de tool0 en el mundo, leida del arbol TF del simulador."""
        limite = time.time() + plazo
        while time.time() < limite:
            self._girar(0.05)
            try:
                t = self._tf.lookup_transform('world', 'station1/tool0',
                                              rclpy.time.Time())
                v = t.transform.translation
                return (v.x, v.y, v.z)
            except Exception:
                continue
        raise RuntimeError('no hay transformada world -> station1/tool0')

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

        resultado = self._esperar(manejador.get_result_async(), T + 25.0)
        t_real = time.time() - t0
        ok = resultado is not None and resultado.result.error_code == 0
        return ok, T, t_real

    def mover_mordaza(self, apertura, duracion=0.6):
        """Abre o cierra el servo de la mordaza.

        Va por su propio controlador, no por el del brazo: el servo no es un
        grado de libertad de la cadena y el planificador no debe verlo nunca.
        La retencion de la pieza no la hace el rozamiento sino el acople
        cinematico de ADR-002; esto anima el servo, que es lo que se ve.
        """
        if self._mordaza is None:
            self._mordaza = ActionClient(
                self, FollowJointTrajectory,
                '/mordaza_controller/follow_joint_trajectory')
        if not self._mordaza.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('el mordaza_controller no responde')
        msg = JointTrajectory()
        msg.joint_names = ['station1/gripper_joint']
        p = JointTrajectoryPoint()
        p.positions = [float(apertura)]
        p.velocities = [0.0]
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
        return self.mover_mordaza(0.5)

    def cerrar_mordaza(self):
        return self.mover_mordaza(0.0)

    def mover_a_punto(self, punto, rama='auto', **kw):
        q, ok, motivo = cin.inversa(punto, rama=rama)
        if not ok:
            self.get_logger().error(
                f'punto {punto} inalcanzable: {motivo}')
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
                    help='punto con nombre de line_poses.yaml')
    ap.add_argument('--ciclo', action='store_true',
                    help='recorre bandeja -> traspaso -> bandeja y mide el tiempo')
    ap.add_argument('--rama', choices=('auto', 'arriba', 'abajo'), default='auto')
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    nodo = ControlPuntoAPunto()
    try:
        if args.ciclo:
            t0 = time.time()
            for nombre in ('bandeja', 'traspaso', 'bandeja'):
                nodo.mover_a_punto(PUNTOS[nombre], rama=args.rama)
            nodo.get_logger().info(
                'ciclo completo en %.1f s, limite de la guia 30 s'
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
