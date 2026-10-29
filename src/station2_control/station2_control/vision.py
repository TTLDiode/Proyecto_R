"""Nodo de vision de la Estacion 2: imagen -> categoria A, B o C por color.

    ros2 run station2_control vision
    ros2 service call /station2/clasificar std_srvs/srv/Trigger

La camara (Camera Module 3 Wide, 102 grados) va bajo el eslabon 2 y mira hacia
abajo. El nodo de la estacion la coloca justo encima de la pieza, de modo que
la pieza ocupa el centro de la imagen: a 97 mm de distancia la pieza de 20 mm
mide unos 50 pixeles de lado en una imagen de 640 x 360.

Metodo, deliberadamente simple y explicable:
  1. Se toma la region central de la imagen (un cuadrado del 40 % de la
     altura), que contiene la pieza y algo del soporte.
  2. Cada pixel se pasa a tono, saturacion y valor (HSV). Los pixeles poco
     saturados o muy oscuros son el soporte gris, la mesa o sombras, y se
     descartan.
  3. Los que quedan se reparten por tono en tres bandas: rojo, verde y azul.
  4. Gana la banda con mas pixeles, si supera un minimo; si no, la pieza no
     se reconoce y el servicio responde success = false.

Tabla de categorias (la misma que usa simulacion.launch.py para crear las
piezas de prueba):
    rojo  -> A        verde -> B        azul  -> C

No usa OpenCV ni cv_bridge: la imagen llega en rgb8 y se decodifica con
numpy, lo que evita una dependencia mas en la maquina de la sustentacion.
"""
import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

CATEGORIAS = {'rojo': 'A', 'verde': 'B', 'azul': 'C'}

# Bandas de tono en grados, [desde, hasta).
BANDAS = {
    'rojo':  ((340.0, 360.0), (0.0, 20.0)),
    'verde': ((80.0, 160.0),),
    'azul':  ((200.0, 260.0),),
}

SAT_MIN = 0.45          # por debajo es gris: soporte, mesa
VAL_MIN = 0.20          # por debajo es sombra
FRACCION_ROI = 0.40     # lado de la region central, sobre la altura
PIXELES_MIN = 40        # menos que esto no es una pieza


def a_hsv(rgb):
    """rgb en [0, 1], forma (..., 3) -> (h en grados, s, v)."""
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    v = rgb.max(axis=-1)
    m = rgb.min(axis=-1)
    c = v - m
    s = np.where(v > 0, c / np.maximum(v, 1e-9), 0.0)
    h = np.zeros_like(v)
    nz = c > 1e-9
    rmax = nz & (v == r)
    gmax = nz & (v == g) & ~rmax
    bmax = nz & ~rmax & ~gmax
    h[rmax] = (60.0 * ((g[rmax] - b[rmax]) / c[rmax])) % 360.0
    h[gmax] = 60.0 * ((b[gmax] - r[gmax]) / c[gmax]) + 120.0
    h[bmax] = 60.0 * ((r[bmax] - g[bmax]) / c[bmax]) + 240.0
    return h, s, v


def clasificar(rgb, fraccion=FRACCION_ROI):
    """Clasifica una imagen rgb (alto, ancho, 3) en [0, 255].

    Devuelve (categoria o None, color o None, cuentas por color).
    """
    alto, ancho, _ = rgb.shape
    lado = int(alto * fraccion)
    y0, x0 = (alto - lado) // 2, (ancho - lado) // 2
    roi = rgb[y0:y0 + lado, x0:x0 + lado].astype(np.float64) / 255.0
    h, s, v = a_hsv(roi)
    vivos = (s >= SAT_MIN) & (v >= VAL_MIN)
    cuentas = {}
    for color, bandas in BANDAS.items():
        en_banda = np.zeros_like(vivos)
        for a, b in bandas:
            en_banda |= (h >= a) & (h < b)
        cuentas[color] = int(np.count_nonzero(vivos & en_banda))
    color = max(cuentas, key=cuentas.get)
    if cuentas[color] < PIXELES_MIN:
        return None, None, cuentas
    return CATEGORIAS[color], color, cuentas


def imagen_a_numpy(msg):
    """sensor_msgs/Image en rgb8 o bgr8 -> array (alto, ancho, 3) rgb."""
    datos = np.frombuffer(msg.data, dtype=np.uint8)
    datos = datos.reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = datos.reshape(msg.height, msg.width, 3)
    if msg.encoding == 'bgr8':
        img = img[..., ::-1]
    elif msg.encoding != 'rgb8':
        raise ValueError('codificacion no soportada: %s' % msg.encoding)
    return img


def guardar_ppm(img, ruta):
    """Guarda la imagen en PPM, que se abre en cualquier visor sin librerias."""
    alto, ancho, _ = img.shape
    with open(ruta, 'wb') as fh:
        fh.write(b'P6\n%d %d\n255\n' % (ancho, alto))
        fh.write(np.ascontiguousarray(img).tobytes())


class NodoVision(Node):

    def __init__(self):
        super().__init__('station2_vision', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('topico_imagen', '/station2/camara/image')
        # Carpeta donde dejar la imagen de cada clasificacion, como
        # evidencia. Vacio para no guardar nada.
        self.declare_parameter('carpeta_evidencia', '')
        self._ultima = None
        self._sello = None
        self._candado = threading.Lock()
        self._n = 0
        self.create_subscription(
            Image, self.get_parameter('topico_imagen').value,
            self._al_llegar_imagen, 2)
        self.create_service(Trigger, '/station2/clasificar', self._clasificar)
        self.get_logger().info('vision lista en /station2/clasificar')

    def _al_llegar_imagen(self, msg):
        with self._candado:
            self._ultima = msg
            self._sello = self.get_clock().now()

    def _clasificar(self, _peticion, respuesta):
        with self._candado:
            msg = self._ultima
        if msg is None:
            respuesta.success = False
            respuesta.message = 'sin imagen de la camara'
            return respuesta
        img = imagen_a_numpy(msg)
        categoria, color, cuentas = clasificar(img)
        self._n += 1
        carpeta = self.get_parameter('carpeta_evidencia').value
        if carpeta:
            os.makedirs(carpeta, exist_ok=True)
            guardar_ppm(img, os.path.join(
                carpeta, 'inspeccion_%02d_%s.ppm' % (self._n, color or 'nada')))
        self.get_logger().info('pixeles por color %s -> %s'
                               % (cuentas, categoria or 'sin reconocer'))
        respuesta.success = categoria is not None
        respuesta.message = categoria or 'pieza no reconocida %s' % cuentas
        return respuesta


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoVision()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
