#!/usr/bin/env python3
"""
Nodo de movimiento punto a punto para la Estacion 3 (RRP).

Recibe un punto (x, y, z) por argumentos de linea de comandos, calcula
la cinematica inversa (station3_control.cinematica), elige la rama
valida (dentro de los limites articulares) y envia la trayectoria al
arm_controller.

Uso:
    ros2 run station3_control punto_a_punto -- --x -0.06 --y -0.14 --z 0.05
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from station3_control.cinematica import cinematica_inversa

NOMBRES_ARTICULACIONES = [
    'station3/joint1_rev',
    'station3/joint2_rev',
    'station3/joint3_prism',
]


class PuntoAPunto(Node):

    def __init__(self, px, py, pz, tiempo_seg):
        super().__init__('station3_punto_a_punto')
        self.publisher = self.create_publisher(
            JointTrajectory, '/station3/arm_controller/joint_trajectory', 10)
        self._enviar(px, py, pz, tiempo_seg)

    def _enviar(self, px, py, pz, tiempo_seg):
        esperas = 0
        while self.publisher.get_subscription_count() == 0 and esperas < 50:
            self.get_logger().info('Esperando a que arm_controller se conecte...')
            rclpy.spin_once(self, timeout_sec=0.1)
            esperas += 1
        self.get_logger().info(
            f'Suscriptores conectados: {self.publisher.get_subscription_count()}')

        soluciones, validas = cinematica_inversa(px, py, pz)

        if not any(validas):
            self.get_logger().error(
                f'Punto ({px}, {py}, {pz}) no alcanzable dentro de los '
                f'limites articulares. Soluciones encontradas: {soluciones}')
            return

        indice = validas.index(True)
        theta1, theta2, s3 = soluciones[indice]
        self.get_logger().info(
            f'Rama {indice + 1} valida: theta1={theta1:.4f} rad, '
            f'theta2={theta2:.4f} rad, s3={s3:.4f} m')

        msg = JointTrajectory()
        msg.joint_names = NOMBRES_ARTICULACIONES
        punto = JointTrajectoryPoint()
        punto.positions = [theta1, theta2, s3]
        punto.time_from_start = Duration(sec=int(tiempo_seg))
        msg.points = [punto]

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Trayectoria enviada a /station3/arm_controller/joint_trajectory '
            f'(objetivo: x={px}, y={py}, z={pz})')


def main(args=None):
    parser = argparse.ArgumentParser(description='Movimiento punto a punto - Estacion 3')
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, required=True)
    parser.add_argument('--tiempo', type=float, default=4.0)
    args_parseados, _ = parser.parse_known_args(
        args if args is not None else sys.argv[1:])

    rclpy.init(args=args)
    nodo = PuntoAPunto(args_parseados.x, args_parseados.y, args_parseados.z,
                        args_parseados.tiempo)
    rclpy.spin_once(nodo, timeout_sec=1.0)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
