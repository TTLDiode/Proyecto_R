"""Orquestador de la linea.

Es el unico nodo que conoce las tres estaciones. Genera los identificadores de
pieza, invoca cada estacion en secuencia, espera su evento de fin y encadena el
resultado hacia la siguiente. Ninguna estacion se suscribe a los eventos de
otra: todo el acoplamiento vive aqui.
"""
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from line_interfaces.msg import HandoffEvent, StationState
from line_interfaces.srv import StartTask

STATIONS = ['station1', 'station2', 'station3']


class Orchestrator(Node):

    def __init__(self):
        super().__init__('orchestrator')

        self.declare_parameter('n_parts', 3)
        self.declare_parameter('start_timeout_s', 10.0)
        self.declare_parameter('done_timeout_s', 60.0)
        self.declare_parameter('poses_file', '')

        self.n_parts = int(self.get_parameter('n_parts').value)
        self.start_timeout = float(self.get_parameter('start_timeout_s').value)
        self.done_timeout = float(self.get_parameter('done_timeout_s').value)

        self.poses = {}
        poses_file = self.get_parameter('poses_file').value
        if poses_file:
            with open(poses_file, 'r') as fh:
                self.poses = yaml.safe_load(fh)

        group = ReentrantCallbackGroup()
        self.clients_ = {}
        self.done_events = {}
        self.last_done = {}
        self.station_state = {}

        for st in STATIONS:
            self.clients_[st] = self.create_client(
                StartTask, f'/line/{st}/start', callback_group=group)
            self.done_events[st] = threading.Event()
            self.create_subscription(
                HandoffEvent, f'/line/{st}/done',
                self._make_done_cb(st), 10, callback_group=group)
            self.create_subscription(
                StationState, f'/line/{st}/state',
                self._make_state_cb(st), 10, callback_group=group)

        self.cycle_times = []

    def _make_done_cb(self, station):
        def cb(msg):
            self.last_done[station] = msg
            self.done_events[station].set()
        return cb

    def _make_state_cb(self, station):
        def cb(msg):
            self.station_state[station] = msg
        return cb

    # -- secuencia de la linea --------------------------------------------

    def run(self):
        for st in STATIONS:
            if not self.clients_[st].wait_for_service(timeout_sec=15.0):
                self.get_logger().error(
                    f'{st} no expone su servicio de inicio. Se aborta la corrida.')
                return False

        tray = self.poses.get('input_tray', {}).get('center', {})
        pose = Pose()
        pose.position.x = float(tray.get('x', 0.0))
        pose.position.y = float(tray.get('y', 0.0))
        pose.position.z = float(tray.get('z', 0.0))
        pose.orientation.w = 1.0

        self.get_logger().info(f'inicio de la corrida: {self.n_parts} piezas')

        for part_id in range(1, self.n_parts + 1):
            t0 = time.monotonic()
            event = HandoffEvent()
            event.part_id = part_id
            event.category = ''
            event.pose = pose
            event.stamp = self.get_clock().now().to_msg()

            for st in STATIONS:
                event = self._run_station(st, event)
                if event is None:
                    self.get_logger().error(
                        f'la pieza {part_id} no completo el ciclo en {st}')
                    return False

            dt = time.monotonic() - t0
            self.cycle_times.append(dt)
            self.get_logger().info(
                f'pieza {part_id} completada en {dt:.1f} s, '
                f'categoria {event.category or "sin asignar"}')

        media = sum(self.cycle_times) / len(self.cycle_times)
        self.get_logger().info(
            f'corrida terminada: {len(self.cycle_times)} piezas, '
            f'tiempo medio de linea {media:.1f} s')
        return True

    def _run_station(self, station, incoming):
        self.done_events[station].clear()

        req = StartTask.Request()
        req.input = incoming
        future = self.clients_[station].call_async(req)

        deadline = time.monotonic() + self.start_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            self.get_logger().error(f'{station} no respondio a la peticion de inicio')
            return None

        result = future.result()
        if not result.accepted:
            self.get_logger().error(f'{station} rechazo la peticion: {result.message}')
            return None

        if not self.done_events[station].wait(timeout=self.done_timeout):
            self.get_logger().error(f'{station} no publico su evento de fin')
            return None

        return self.last_done[station]


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # El ejecutor debe detenerse y su hilo unirse antes de destruir el nodo:
        # de lo contrario el hilo sigue trabajando sobre estructuras liberadas y
        # el proceso termina con SIGABRT.
        executor.shutdown()
        spin_thread.join(timeout=5.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
