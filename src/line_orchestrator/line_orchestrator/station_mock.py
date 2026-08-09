"""Nodo de prueba de estacion.

Implementa el protocolo de comunicacion definido en la documentacion sin
accionar ningun manipulador: acepta la peticion de inicio, espera un tiempo
configurable, publica el evento de fin con la pose de salida que corresponde a
su estacion y mantiene la publicacion de estado a 2 Hz.

Sirve para validar el protocolo y el orquestador por separado del control, de
modo que un fallo posterior pueda atribuirse a un cambio concreto.
"""
import threading

import rclpy
import yaml
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from line_interfaces.msg import HandoffEvent, StationState
from line_interfaces.srv import StartTask


def _pose_from(d):
    p = Pose()
    p.position.x = float(d.get('x', 0.0))
    p.position.y = float(d.get('y', 0.0))
    p.position.z = float(d.get('z', 0.0))
    p.orientation.w = 1.0
    return p


class StationMock(Node):

    def __init__(self):
        super().__init__('station_mock')

        self.declare_parameter('station_id', 'station1')
        self.declare_parameter('duration_s', 3.0)
        self.declare_parameter('assign_category', False)
        self.declare_parameter('poses_file', '')

        self.station_id = self.get_parameter('station_id').value
        self.duration_s = float(self.get_parameter('duration_s').value)
        self.assign_category = bool(self.get_parameter('assign_category').value)

        self.poses = {}
        poses_file = self.get_parameter('poses_file').value
        if poses_file:
            with open(poses_file, 'r') as fh:
                self.poses = yaml.safe_load(fh)

        self.state = StationState.IDLE
        self.detail = 'nodo de prueba'
        self._lock = threading.Lock()

        group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            StartTask, f'/line/{self.station_id}/start',
            self.on_start, callback_group=group)
        self.pub_done = self.create_publisher(
            HandoffEvent, f'/line/{self.station_id}/done', 10)
        self.pub_state = self.create_publisher(
            StationState, f'/line/{self.station_id}/state', 10)
        self.create_timer(0.5, self.publish_state, callback_group=group)

        self.get_logger().info(
            f'nodo de prueba de {self.station_id} listo, '
            f'ciclo de {self.duration_s:.1f} s')

    # -- protocolo ---------------------------------------------------------

    def on_start(self, request, response):
        with self._lock:
            if self.state == StationState.BUSY:
                response.accepted = False
                response.message = 'la estacion esta ocupada'
                return response
            self.state = StationState.BUSY
            self.detail = f'procesando la pieza {request.input.part_id}'

        response.accepted = True
        response.message = 'aceptada'
        threading.Thread(
            target=self._run_cycle, args=(request.input,), daemon=True).start()
        return response

    def _run_cycle(self, incoming):
        self.get_logger().info(
            f'{self.station_id}: pieza {incoming.part_id} recibida')
        threading.Event().wait(self.duration_s)

        out = HandoffEvent()
        out.part_id = incoming.part_id
        out.category = incoming.category
        if self.assign_category and not out.category:
            out.category = ['A', 'B', 'C'][incoming.part_id % 3]
        out.pose = self._output_pose(out.category)
        out.stamp = self.get_clock().now().to_msg()

        with self._lock:
            self.state = StationState.IDLE
            self.detail = 'a la espera'

        self.pub_done.publish(out)
        self.get_logger().info(
            f'{self.station_id}: pieza {out.part_id} entregada'
            + (f', categoria {out.category}' if out.category else ''))

    def _output_pose(self, category):
        """Pose de salida segun la posicion de la estacion en la linea."""
        if self.station_id == 'station1':
            return _pose_from(self.poses.get('handoff_1_2', {}).get('position', {}))
        if self.station_id == 'station2':
            return _pose_from(self.poses.get('handoff_2_3', {}).get('position', {}))
        slots = self.poses.get('packing', {}).get('box_slots', {})
        return _pose_from(slots.get(category or 'A', {}))

    def publish_state(self):
        msg = StationState()
        with self._lock:
            msg.state = self.state
            msg.detail = self.detail
        msg.station_id = self.station_id
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StationMock()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
