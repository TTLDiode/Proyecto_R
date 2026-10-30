"""Reposición automática de la pieza en la bandeja de entrada.

Tras cada /line/station3/done elimina el modelo "pieza" y spawnea uno nuevo
en la bandeja, rotando color rojo/verde/azul (categorías A/B/C).

    ros2 run line_orchestrator reponedor
"""

import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from line_interfaces.msg import HandoffEvent

X, Y, Z = -0.180, 0.100, 0.038
WORLD = "line"
MODEL = "pieza"

COLORES = [
    ("0.85 0.10 0.10 1", "0.95 0.15 0.15 1", "A"),  # rojo
    ("0.10 0.70 0.15 1", "0.15 0.85 0.20 1", "B"),  # verde
    ("0.10 0.15 0.80 1", "0.15 0.20 0.90 1", "C"),  # azul
]


def sdf_pieza(ambient, diffuse):
    return f'''<sdf version="1.9">
  <model name="{MODEL}">
    <pose>{X} {Y} {Z} 0 0 0</pose>
    <link name="cuerpo">
      <inertial>
        <mass>0.020</mass>
        <inertia>
          <ixx>9.067e-07</ixx><iyy>9.067e-07</iyy><izz>1.333e-06</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.020 0.020 0.012</size></box></geometry>
        <surface>
          <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.020 0.020 0.012</size></box></geometry>
        <material>
          <ambient>{ambient}</ambient>
          <diffuse>{diffuse}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''


def run(cmd, timeout=8):
    try:
        return subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None


def eliminar_pieza():
    req = f'name: "{MODEL}", type: MODEL'
    return run([
        "gz", "service", "-s", f"/world/{WORLD}/remove",
        "--reqtype", "gz.msgs.Entity",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", req,
    ]) is not None


def spawn_pieza(ambient, diffuse):
    sdf = sdf_pieza(ambient, diffuse)
    r = run([
        "ros2", "run", "ros_gz_sim", "create",
        "-world", WORLD,
        "-name", MODEL,
        "-string", sdf,
    ], timeout=15)
    return r is not None and r.returncode == 0


class Reponedor(Node):

    def __init__(self):
        super().__init__("reponedor_pieza")
        self._i = 0
        self._lock = threading.Lock()
        self._pub_detach_s1 = self.create_publisher(
            Empty, "/station1/pieza/detach", 10)
        self._pub_detach_s2 = self.create_publisher(
            Empty, "/station2/pieza/detach", 10)
        self._pub_detach_s3 = self.create_publisher(
            Empty, "/station3/pieza/detach", 10)
        self.create_subscription(
            HandoffEvent, "/line/station3/done", self._on_done, 10)
        self.get_logger().info(
            "Reponedor listo: tras cada /line/station3/done repone la pieza")

    def _on_done(self, msg: HandoffEvent):
        if not self._lock.acquire(blocking=False):
            return
        try:
            self._reponer(msg.part_id)
        finally:
            self._lock.release()

    def _reponer(self, part_id):
        ambient, diffuse, cat = COLORES[self._i % len(COLORES)]
        self._i += 1
        self.get_logger().info(
            f"Ciclo {part_id} terminado → reponiendo pieza color {cat}")

        empty = Empty()
        self._pub_detach_s1.publish(empty)
        self._pub_detach_s2.publish(empty)
        self._pub_detach_s3.publish(empty)
        time.sleep(0.5)

        eliminar_pieza()
        time.sleep(0.3)

        if spawn_pieza(ambient, diffuse):
            self.get_logger().info(
                f"Pieza nueva en bandeja ({X}, {Y}, {Z}), categoría esperada {cat}")
        else:
            self.get_logger().error(
                "No se pudo spawnear la pieza (¿Gazebo sigue vivo?)")


def main(args=None):
    rclpy.init(args=args)
    node = Reponedor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()