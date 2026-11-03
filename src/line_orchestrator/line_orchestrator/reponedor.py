"""Reposición automática de la pieza en la bandeja de entrada.

Tras cada /line/station3/done deja en la caja una copia fija (solo visual)
de la pieza, donde la dejo la E3, elimina el modelo "pieza" y spawnea uno
nuevo en la bandeja, rotando color rojo/azul/verde (categorías A/C/B). Las
tres estaciones acoplan siempre el modelo "pieza", por eso se recicla y lo
que queda en las cajas son copias.

    ros2 run line_orchestrator reponedor
"""

import re
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

# La primera pieza (la de line.sdf) es verde; despues rojo, azul y verde,
# para que cada corrida de 3 piezas tenga los tres colores.
COLORES = [
    ("0.85 0.10 0.10 1", "0.95 0.15 0.15 1", "A"),  # rojo
    ("0.10 0.15 0.80 1", "0.15 0.20 0.90 1", "C"),  # azul
    ("0.10 0.70 0.15 1", "0.15 0.85 0.20 1", "B"),  # verde
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


# Color de la pieza que trae line.sdf (la primera de la corrida).
COLOR_INICIAL = ("0.10 0.70 0.15 1", "0.15 0.85 0.20 1")


def pose_pieza():
    """(x, y, z, qx, qy, qz, qw) del modelo "pieza" en el mundo, o None."""
    r = run(["gz", "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "-n", "1"])
    if r is None or r.returncode != 0:
        return None
    m = re.search(r'name: "%s"\s+id: \d+\s+position \{([^}]*)\}\s+'
                  r'orientation \{([^}]*)\}' % MODEL, r.stdout)
    if not m:
        return None

    def campos(texto, nombres, defecto):
        valores = dict(re.findall(r'(\w+): ([-\d.e+]+)', texto))
        return [float(valores.get(n, defecto)) for n in nombres]
    x, y, z = campos(m.group(1), "xyz", 0.0)
    qx, qy, qz = campos(m.group(2), "xyz", 0.0)
    qw, = campos(m.group(2), "w", 0.0)
    return x, y, z, qx, qy, qz, qw


def dejar_copia(nombre, pose, ambient, diffuse):
    """Copia estatica y sin colision de la pieza: marca donde quedo sin
    estorbar a las piezas siguientes."""
    x, y, z, qx, qy, qz, qw = pose
    sdf = f'''<sdf version="1.9">
  <model name="{nombre}">
    <static>true</static>
    <link name="cuerpo">
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
    r = run([
        "gz", "service", "-s", f"/world/{WORLD}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", f'sdf: {sdf!r} pose: {{position: {{x: {x}, y: {y}, z: {z}}} '
                 f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}',
    ])
    return r is not None and "true" in r.stdout


def spawn_pieza(ambient, diffuse):
    sdf = sdf_pieza(ambient, diffuse)
    r = run([
        "ros2", "run", "ros_gz_sim", "create",
        "-world", WORLD,
        "-name", MODEL,
        "-string", sdf,
        # Sin -x/-y/-z, create ignora la <pose> del SDF y la pieza nace en
        # el origen (dentro de la base de la E1).
        "-x", str(X), "-y", str(Y), "-z", str(Z),
    ], timeout=15)
    return r is not None and r.returncode == 0


class Reponedor(Node):

    def __init__(self):
        super().__init__("reponedor_pieza")
        self._i = 0
        # 0 = sin limite. Con n_parts, tras la ultima pieza no se repone otra.
        self.declare_parameter("n_parts", 0)
        self._n_parts = int(self.get_parameter("n_parts").value)
        self._color_actual = COLOR_INICIAL
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
        self.get_logger().info(f"Ciclo {part_id} terminado")

        empty = Empty()
        self._pub_detach_s1.publish(empty)
        self._pub_detach_s2.publish(empty)
        self._pub_detach_s3.publish(empty)
        time.sleep(0.5)

        pose = pose_pieza()
        if pose and dejar_copia(f"pieza_caja_{part_id}", pose,
                                *self._color_actual):
            self.get_logger().info(
                f"Pieza {part_id} queda en ({pose[0]:.3f}, {pose[1]:.3f})")
        else:
            self.get_logger().warn(f"No se pudo dejar la copia de la pieza {part_id}")

        eliminar_pieza()
        time.sleep(0.3)

        if self._n_parts and part_id >= self._n_parts:
            self.get_logger().info(
                f"Pieza {part_id} de {self._n_parts}: fin, no se repone otra")
            return

        if spawn_pieza(ambient, diffuse):
            self._color_actual = (ambient, diffuse)
            # Los acoples nacen enganchados a la pieza nueva: se sueltan.
            time.sleep(0.5)
            self._pub_detach_s1.publish(empty)
            self._pub_detach_s2.publish(empty)
            self._pub_detach_s3.publish(empty)
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