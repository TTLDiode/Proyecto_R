"""Prueba aislada de la Estacion 2 con las tres piezas de colores.

    ros2 launch station2_description simulacion.launch.py gui:=false
    ros2 run station2_control vision
    ros2 run station2_control nodo_estacion
    ros2 run station2_control prueba_estacion
    ros2 run station2_control prueba_estacion --salida resultados/estacion2

Para cada pieza (roja, verde y azul):
  1. la coloca sobre el soporte del traspaso 1 -> 2, como la dejaria la
     Estacion 1;
  2. le dice al nodo de la estacion que pieza acoplar y le pide un ciclo por
     /line/station2/start, igual que el orquestador;
  3. espera el evento /line/station2/done y anota la categoria;
  4. lee de Gazebo donde quedo la pieza y mide su error contra el punto de
     traspaso 2 -> 3, que es la precision que pide la guia (< 20 mm);
  5. retira la pieza del soporte para dejar sitio a la siguiente.

Escribe la tabla en <salida>/prueba_estacion.md.

Las piezas se mueven con el servicio set_pose del mundo de Gazebo, por la
linea de comandos de gz, para no depender de un puente de servicios mas.
"""
import argparse
import math
import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from line_interfaces.msg import HandoffEvent
from line_interfaces.srv import StartTask
from rcl_interfaces.msg import Parameter as ParametroMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

from station2_control.punto_a_punto import PUNTOS

ESPERADO = {'pieza_roja': 'A', 'pieza_verde': 'B', 'pieza_azul': 'C'}
APARCAMIENTO = {
    'pieza_roja':  (0.40, -0.30, 0.006),
    'pieza_verde': (0.50, -0.30, 0.006),
    'pieza_azul':  (0.60, -0.30, 0.006),
}
# Centro de la pieza de 12 mm apoyada en un soporte de 42 mm.
Z_PIEZA = 0.048
MUNDO = 'line'


def gz_mover(nombre, x, y, z):
    req = ('name: "%s", position: {x: %f, y: %f, z: %f}, '
           'orientation: {w: 1}' % (nombre, x, y, z))
    subprocess.run(['gz', 'service', '-s', '/world/%s/set_pose' % MUNDO,
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000', '--req', req],
                   check=True, capture_output=True)


def gz_pose(nombre):
    """Posicion de un modelo, leida de la linea de comandos de gz."""
    salida = subprocess.run(['gz', 'model', '-m', nombre, '-p'],
                            capture_output=True, text=True,
                            timeout=20).stdout
    m = re.search(r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]+)\]',
                  salida)
    if not m:
        raise RuntimeError('no pude leer la pose de %s:\n%s' % (nombre, salida))
    return tuple(float(v) for v in m.group(1).split()[:3])


class Prueba(Node):

    def __init__(self):
        super().__init__('station2_prueba')
        self._inicio = self.create_client(StartTask, '/line/station2/start')
        self._param = self.create_client(
            SetParameters, '/station2_nodo_estacion/set_parameters')
        self._fin = threading.Event()
        self._evento = None
        self.create_subscription(HandoffEvent, '/line/station2/done',
                                 self._al_terminar, 10)

    def _al_terminar(self, msg):
        self._evento = msg
        self._fin.set()

    def _llamar(self, cliente, peticion, plazo=10.0):
        if not cliente.wait_for_service(timeout_sec=plazo):
            raise RuntimeError('no responde %s' % cliente.srv_name)
        futuro = cliente.call_async(peticion)
        limite = time.time() + plazo
        while not futuro.done() and time.time() < limite:
            time.sleep(0.01)
        return futuro.result()

    def fijar_pieza(self, nombre):
        p = ParametroMsg()
        p.name = 'pieza'
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                 string_value=nombre)
        self._llamar(self._param, SetParameters.Request(parameters=[p]))

    def ciclo(self, part_id, plazo=240.0):
        self._fin.clear()
        pet = StartTask.Request()
        pet.input.part_id = part_id
        r = self._llamar(self._inicio, pet)
        if not r or not r.accepted:
            raise RuntimeError('la estacion rechazo el ciclo: %s'
                               % (r.message if r else 'sin respuesta'))
        t0 = time.time()
        if not self._fin.wait(plazo):
            raise RuntimeError('la estacion no publico done en %.0f s' % plazo)
        return self._evento, time.time() - t0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--salida', default='resultados/estacion2')
    ap.add_argument('--piezas', default='pieza_roja,pieza_verde,pieza_azul')
    args = ap.parse_args(argv if argv is not None else
                         rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    nodo = Prueba()
    hilo = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    hilo.start()
    destino = PUNTOS['traspaso_23']
    filas = []
    try:
        for i, pieza in enumerate(args.piezas.split(','), start=1):
            origen = PUNTOS['traspaso_12']
            gz_mover(pieza, origen[0], origen[1], Z_PIEZA)
            time.sleep(1.0)
            nodo.fijar_pieza(pieza)
            evento, dur = nodo.ciclo(i)
            time.sleep(1.0)
            p = gz_pose(pieza)
            # La pieza se compara por su centro en planta y por su altura
            # respecto de la que tendria apoyada en el soporte.
            err_xy = math.hypot(p[0] - destino[0], p[1] - destino[1])
            err_z = abs(p[2] - Z_PIEZA)
            fila = (pieza, ESPERADO[pieza], evento.category, dur, p,
                    err_xy, err_z)
            filas.append(fila)
            nodo.get_logger().info(
                '%s: esperada %s, asignada %s, ciclo %.1f s, pieza en '
                '(%.6f, %.6f, %.6f), error en planta %.3f mm'
                % (pieza, fila[1], fila[2], dur, *p, err_xy*1000))
            gz_mover(pieza, *APARCAMIENTO[pieza])
            time.sleep(1.0)
    finally:
        os.makedirs(args.salida, exist_ok=True)
        ruta = os.path.join(args.salida, 'prueba_estacion.md')
        with open(ruta, 'w') as fh:
            fh.write('# Prueba aislada de la Estacion 2\n\n'
                     'Generado por `ros2 run station2_control prueba_estacion`.'
                     ' Pieza colocada en el traspaso 1-2 (0.250, 0.000, %.3f);'
                     ' destino traspaso 2-3 (%.3f, %.3f), centro de la pieza a'
                     ' z = %.3f.\n\n' % (Z_PIEZA, destino[0], destino[1],
                                         Z_PIEZA))
            fh.write('| pieza | categoria esperada | categoria asignada | '
                     'ciclo (s) | posicion final (m) | error en planta (mm) | '
                     'error en altura (mm) |\n|---|---|---|---|---|---|---|\n')
            for f in filas:
                fh.write('| %s | %s | %s | %.1f | (%.6f, %.6f, %.6f) | %.3f | '
                         '%.3f |\n' % (f[0], f[1], f[2], f[3], *f[4],
                                       f[5]*1000, f[6]*1000))
            if filas:
                bien = sum(1 for f in filas if f[1] == f[2])
                peor = max(f[5] for f in filas)
                fh.write('\nClasificacion correcta: %d de %d. Peor error de '
                         'posicionamiento en planta: %.3f mm (limite de la '
                         'guia: 20 mm).\n' % (bien, len(filas), peor*1000))
        nodo.get_logger().info('tabla escrita en %s' % ruta)
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
