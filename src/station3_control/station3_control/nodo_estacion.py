#!/usr/bin/env python3
"""
Nodo de la Estacion 3 (RRP): clasificacion y empaque.

Implementa el contrato de linea (docs/03-contrato-de-interfaz.md):
  /line/station3/start  (StartTask)       - recibe la pieza, arranca el ciclo
  /line/station3/done   (HandoffEvent)    - se publica una vez al terminar
  /line/station3/state  (StationState)    - latido continuo a 2 Hz

Ciclo: recibir en handoff_2_3 -> acoplar -> llevar a la caja segun
categoria (contando 10 por caja, nueva caja al completar) -> desacoplar.

Las posiciones (handoff, cajas, base propia) se leen de line_poses.yaml,
igual que hace line_orchestrator/station_mock.py, para no duplicar
numeros que el equipo ya acordo en un solo lugar.
"""

import math
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration, Time
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from line_interfaces.msg import HandoffEvent, StationState
from line_interfaces.srv import StartTask

from station3_control.cinematica import cinematica_inversa

NOMBRES_ARTICULACIONES = [
    'station3/joint1_rev',
    'station3/joint2_rev',
    'station3/joint3_prism',
]

Z_APROXIMACION = 0.060
Z_AGARRE = 0.017
TIEMPO_MOVIMIENTO = 3.0

# Desplazamientos de deposito, uno por pieza, RELATIVOS al centro de cada
# caja (que se lee de line_poses.yaml en tiempo real, no se fija aqui).
# Con la base y las cajas de hoy, el delta (0,0) coincide con el centro de la
# caja solo en la caja A; en B y C los deltas parten de un punto distinto del
# centro, para reproducir la rejilla acordada. Cada caja admite 10 piezas.
# Si algun dia se mueve la base de la estacion o las cajas en el yaml,
# esta rejilla se mueve junto con ellas sin tocar este archivo.
DELTAS_SLOT = {
    'A': [(0.0, 0.0), (0.02, 0.0), (0.04, 0.0), (0.06, 0.0), (0.08, 0.0),
          (0.0, -0.02), (0.02, -0.02), (0.04, -0.02), (0.06, -0.02), (0.08, -0.02)],
    'B': [(0.0, 0.04), (0.0, 0.02), (0.0, 0.0), (0.0, -0.02), (0.0, -0.04),
          (0.02, 0.04), (0.02, 0.02), (0.02, 0.0), (0.02, -0.02), (0.02, -0.04)],
    'C': [(0.0, 0.02), (0.02, 0.02), (0.04, 0.02), (0.06, 0.02), (0.08, 0.02),
          (0.0, 0.0), (0.02, 0.0), (0.04, 0.0), (0.06, 0.0), (0.08, 0.0)],
}


def mundo_a_local(wx, wy, wz, base_x, base_y, yaw_deg):
    dx, dy = wx - base_x, wy - base_y
    yaw = math.radians(yaw_deg)
    lx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
    ly = math.sin(-yaw) * dx + math.cos(-yaw) * dy
    return lx, ly, wz


def local_a_mundo(lx, ly, lz, base_x, base_y, yaw_deg):
    yaw = math.radians(yaw_deg)
    wx = base_x + math.cos(yaw) * lx - math.sin(yaw) * ly
    wy = base_y + math.sin(yaw) * lx + math.cos(yaw) * ly
    return wx, wy, lz


class NodoEstacion3(Node):

    def __init__(self):
        super().__init__('station3_nodo_estacion')

        self.declare_parameter('poses_file', '')
        poses_file = self.get_parameter('poses_file').value
        if not poses_file:
            poses_file = (
                get_package_share_directory('line_bringup')
                + '/config/line_poses.yaml')
        with open(poses_file, 'r') as f:
            self.poses = yaml.safe_load(f)

        base = self.poses['station_bases']['station3']
        self.base_x = base['x']
        self.base_y = base['y']
        self.yaw = base['yaw']

        self.conteo_por_categoria = {}
        self.unidades_por_caja = self.poses['constraints']['station3_units_per_box']

        self.pub_trayectoria = self.create_publisher(
            JointTrajectory, '/station3/arm_controller/joint_trajectory', 10)
        self.pub_attach = self.create_publisher(Empty, '/station3/pieza/attach', 10)
        self.pub_detach = self.create_publisher(Empty, '/station3/pieza/detach', 10)
        self.pub_estado = self.create_publisher(StationState, '/line/station3/state', 10)
        self.pub_done = self.create_publisher(HandoffEvent, '/line/station3/done', 10)
        self.cliente_traj = ActionClient(
            self, FollowJointTrajectory,
            '/station3/arm_controller/follow_joint_trajectory')

        self.ocupado = False
        self.timer_estado = self.create_timer(0.5, self._publicar_estado)

        self.srv_start = self.create_service(
            StartTask, '/line/station3/start', self._on_start)

        self.get_logger().info('Nodo de Estacion 3 listo.')

    def _esperar(self, future, timeout=60.0):
        """Bloquea este hilo hasta que el future termine. Devuelve su
        resultado, o None si se agota el timeout."""
        evento = threading.Event()
        future.add_done_callback(lambda _: evento.set())
        if not evento.wait(timeout):
            return None
        return future.result()

    def _mover(self, px, py, pz):
        """Mueve el efector al punto local y espera a que el controlador
        confirme que llego. Lanza RuntimeError si algo falla."""
        soluciones, validas = cinematica_inversa(px, py, pz)
        if not any(validas):
            raise RuntimeError(
                f'Punto local ({px:.3f},{py:.3f},{pz:.3f}) no alcanzable.')
        theta1, theta2, s3 = soluciones[validas.index(True)]

        if not self.cliente_traj.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('Servidor de trayectoria no disponible.')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = NOMBRES_ARTICULACIONES
        punto = JointTrajectoryPoint()
        punto.positions = [theta1, theta2, s3]
        punto.time_from_start = Duration(sec=int(TIEMPO_MOVIMIENTO))
        goal.trajectory.points = [punto]

        goal_handle = self._esperar(self.cliente_traj.send_goal_async(goal))
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('El controlador rechazo la trayectoria.')

        resultado = self._esperar(goal_handle.get_result_async())
        if (resultado is None or resultado.result.error_code
                != FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError('El movimiento no termino con exito.')

    def _slot_local(self, categoria):
        """Punto local (x, y) donde se deposita la siguiente pieza de la
        categoria, segun cuantas lleva ya. El centro de la caja se lee del
        yaml en cada llamada, asi que si la base o las cajas se mueven en
        linea_poses.yaml, la rejilla de 10 slots se mueve con ellas."""
        slot = self.poses['packing']['box_slots'][categoria]
        centro_lx, centro_ly, _ = mundo_a_local(
            slot['x'], slot['y'], 0.0, self.base_x, self.base_y, self.yaw)

        deltas = DELTAS_SLOT[categoria]
        cuenta = self.conteo_por_categoria.get(categoria, 0)
        dx, dy = deltas[cuenta % len(deltas)]
        return centro_lx + dx, centro_ly + dy

    def _on_start(self, request, response):
        if self.ocupado:
            response.accepted = False
            response.message = 'Estacion 3 ocupada.'
            return response

        response.accepted = True
        response.message = 'Ciclo iniciado.'
        self.ocupado = True
        hilo = threading.Thread(
            target=self._ciclo, args=(request.input,), daemon=True)
        hilo.start()
        return response

    def _ciclo(self, pieza: HandoffEvent):
        try:
            self._ciclo_interno(pieza)
        except Exception as e:
            self.get_logger().error(f'Ciclo abortado: {e}')
        finally:
            self.ocupado = False

    def _ciclo_interno(self, pieza: HandoffEvent):
        categoria = pieza.category
        self.get_logger().info(
            f'Iniciando ciclo para pieza {pieza.part_id}, categoria {categoria}')

        # 1. Ir sobre el punto de traspaso, bajar, acoplar, subir
        hx = self.poses['handoff_2_3']['position']['x']
        hy = self.poses['handoff_2_3']['position']['y']
        lx, ly, _ = mundo_a_local(hx, hy, 0.0, self.base_x, self.base_y, self.yaw)

        self._mover(lx, ly, Z_APROXIMACION)
        self._mover(lx, ly, Z_AGARRE)
        self.pub_attach.publish(Empty())
        time.sleep(0.3)
        self._mover(lx, ly, Z_APROXIMACION)

        # 2. Ir sobre la caja correspondiente, bajar, desacoplar, subir
        lx, ly = self._slot_local(categoria)
        bx, by, bz = local_a_mundo(
            lx, ly, Z_AGARRE, self.base_x, self.base_y, self.yaw)

        self._mover(lx, ly, Z_APROXIMACION)
        self._mover(lx, ly, Z_AGARRE)
        self.pub_detach.publish(Empty())
        time.sleep(0.3)
        self._mover(lx, ly, Z_APROXIMACION)

        self.conteo_por_categoria[categoria] = (
            self.conteo_por_categoria.get(categoria, 0) + 1)

        # 3. Publicar el evento de fin, con la pose de salida (la caja)
        salida = HandoffEvent()
        salida.part_id = pieza.part_id
        salida.category = categoria
        salida.pose.position.x = bx
        salida.pose.position.y = by
        salida.pose.position.z = bz
        salida.stamp = self.get_clock().now().to_msg()
        self.pub_done.publish(salida)

        self.get_logger().info(
            f'Ciclo completo para {pieza.part_id}. '
            f'Unidades en categoria {categoria}: '
            f'{self.conteo_por_categoria[categoria]}')

    def _publicar_estado(self):
        msg = StationState()
        msg.state = StationState.BUSY if self.ocupado else StationState.IDLE
        msg.station_id = 'station3'
        msg.detail = ''
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_estado.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoEstacion3()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
