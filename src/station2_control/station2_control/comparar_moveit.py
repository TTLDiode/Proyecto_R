"""Contraste entre MoveIt y el codigo propio sobre los puntos de tarea.

    ros2 launch station2_description simulacion.launch.py gui:=false
    ros2 launch station2_moveit_config move_group.launch.py
    ros2 run station2_control comparar_moveit
    ros2 run station2_control comparar_moveit --salida resultados/estacion2

Adaptado de station1_control/comparar_moveit.py. Para cada punto mide, por
las dos vias:

  - la solucion articular y cuanto se separan entre si
  - el error de la solucion contra la cinematica directa
  - el tiempo que tarda en obtenerla
  - el tiempo de planificacion de MoveIt
  - el error de la pose alcanzada, leida del arbol TF del simulador
  - la duracion del movimiento

Las dos vias parten siempre del mismo estado (reposo), al que se vuelve entre
medida y medida.

Lo propio de la Estacion 2: los dos traspasos caen en ramas de codo
distintas. La inversa propia elige la rama a sabiendas (ver
cinematica.inversa); KDL no conoce las ramas y converge a la que tenga mas
cerca de la semilla. Por eso la tabla anota la rama de cada solucion.
"""
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions, PositionIKRequest, RobotState)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from station2_control import cinematica as cin
from station2_control import trayectoria as tra
from station2_control.punto_a_punto import PUNTOS, ControlPuntoAPunto

# move_group corre en el espacio de nombres de la estacion.
ESPACIO = '/station2'

# Estado comun de partida: el reposo del nodo de la estacion.
REPOSO = (math.radians(90.0), 0.0, 0.0)

# Puntos de la prueba, en el frame world. Los tres de la tarea y dos mas del
# espacio de trabajo, a otras alturas, para no medir solo z = 0.060.
PUNTOS_PRUEBA = (
    ('traspaso_12', PUNTOS['traspaso_12']),
    ('inspeccion', PUNTOS['inspeccion']),
    ('traspaso_23', PUNTOS['traspaso_23']),
    ('libre_1', (0.400, 0.250, 0.080)),
    ('libre_2', (0.700, 0.150, 0.100)),
)


def _rama(q):
    return 'arriba' if q[1] > 0 else 'abajo'


class Comparador(ControlPuntoAPunto):

    def __init__(self):
        super().__init__('station2_comparar_moveit')
        self._ik = self.create_client(GetPositionIK, ESPACIO + '/compute_ik')
        self._plan = ActionClient(self, MoveGroup, ESPACIO + '/move_action')
        self._ejecutar = ActionClient(self, ExecuteTrajectory,
                                      ESPACIO + '/execute_trajectory')

    # ---------------------------------------------------------------
    def estado_actual(self):
        q = self.esperar_estado()
        est = RobotState()
        est.joint_state = JointState()
        est.joint_state.name = list(cin.ARTICULACIONES)
        est.joint_state.position = [float(v) for v in q]
        est.is_diff = False
        return est

    def ik_moveit(self, punto, plazo=1.0):
        """Cinematica inversa de MoveIt. Devuelve (q, segundos, codigo)."""
        if not self._ik.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('move_group no ofrece compute_ik')
        pet = GetPositionIK.Request()
        pet.ik_request = PositionIKRequest()
        pet.ik_request.group_name = 'brazo'
        pet.ik_request.ik_link_name = 'station2/tool0'
        pet.ik_request.robot_state = self.estado_actual()
        pet.ik_request.avoid_collisions = True
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.pose = Pose()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = punto
        # Con position_only_ik la orientacion se ignora.
        pose.pose.orientation.w = 1.0
        pet.ik_request.pose_stamped = pose
        pet.ik_request.timeout.sec = int(plazo)
        pet.ik_request.timeout.nanosec = int((plazo % 1) * 1e9)

        t0 = time.perf_counter()
        r = self._esperar(self._ik.call_async(pet), 15.0)
        dt = time.perf_counter() - t0
        if r is None or r.error_code.val != 1:
            return None, dt, (r.error_code.val if r else None)
        nombres = list(r.solution.joint_state.name)
        q = [r.solution.joint_state.position[nombres.index(j)]
             for j in cin.ARTICULACIONES]
        return q, dt, 1

    def planificar(self, q_objetivo, plazo=5.0):
        """Solo planifica. Devuelve (trayectoria, segundos de planificacion)."""
        if not self._plan.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('no responde la accion move_action')
        meta = MoveGroup.Goal()
        pet = MotionPlanRequest()
        pet.group_name = 'brazo'
        pet.allowed_planning_time = plazo
        pet.num_planning_attempts = 10
        pet.max_velocity_scaling_factor = 1.0
        pet.max_acceleration_scaling_factor = 1.0
        pet.start_state = self.estado_actual()
        restricciones = Constraints()
        for nombre, valor in zip(cin.ARTICULACIONES, q_objetivo):
            restricciones.joint_constraints.append(
                JointConstraint(joint_name=nombre, position=float(valor),
                                tolerance_above=1e-4, tolerance_below=1e-4,
                                weight=1.0))
        pet.goal_constraints.append(restricciones)
        meta.request = pet
        meta.planning_options = PlanningOptions(plan_only=True)

        t0 = time.perf_counter()
        manejador = self._esperar(self._plan.send_goal_async(meta), 30.0)
        if manejador is None or not manejador.accepted:
            return None, 0.0
        r = self._esperar(manejador.get_result_async(), plazo + 30.0)
        dt = time.perf_counter() - t0
        if r is None or r.result.error_code.val != 1:
            return None, dt
        return r.result.planned_trajectory, dt

    def ejecutar_trayectoria(self, trayectoria, plazo=90.0):
        """Ejecuta una trayectoria ya planificada. Devuelve (ok, segundos)."""
        if not self._ejecutar.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('no responde la accion execute_trajectory')
        meta = ExecuteTrajectory.Goal()
        meta.trajectory = trayectoria
        t0 = time.perf_counter()
        manejador = self._esperar(self._ejecutar.send_goal_async(meta), 30.0)
        if manejador is None or not manejador.accepted:
            return False, 0.0
        r = self._esperar(manejador.get_result_async(), plazo)
        dt = time.perf_counter() - t0
        return (r is not None and r.result.error_code.val == 1), dt

    # ---------------------------------------------------------------
    def volver_a_reposo(self):
        self.mover_a_articular(REPOSO)
        time.sleep(0.3)

    def medir(self, nombre, punto):
        fila = {'punto': nombre, 'objetivo': punto}
        base = cin.a_base(punto)

        # --- via propia ---
        self.volver_a_reposo()
        t0 = time.perf_counter()
        q_propia, ok, motivo = cin.inversa(base)
        fila['t_ik_propia'] = time.perf_counter() - t0
        if not ok:
            fila['motivo'] = motivo
            return fila
        fila['q_propia'] = q_propia
        fila['err_ik_propia'] = cin.error_cierre(base, q_propia)
        q_ini = self.esperar_estado()
        fila['dur_propia'] = tra.duracion(q_ini, q_propia)
        exito, _, t_real = self.mover_a_articular(q_propia)
        fila['ok_propia'] = exito
        fila['t_eje_propia'] = t_real
        fila['err_propia'] = math.dist(self.pose_herramienta(), punto)

        # --- via MoveIt ---
        self.volver_a_reposo()
        q_moveit, t_ik, cod = self.ik_moveit(punto)
        fila['t_ik_moveit'] = t_ik
        fila['q_moveit'] = q_moveit
        if q_moveit is None:
            fila['cod_moveit'] = cod
            return fila
        fila['err_ik_moveit'] = cin.error_cierre(base, q_moveit)
        # Se compara contra la solucion propia de la misma rama, si existe.
        q_misma, ok_misma, _ = cin.inversa(base, rama=_rama(q_moveit))
        ref = q_misma if ok_misma else q_propia
        fila['dq'] = max(abs(a - b) for a, b in zip(ref, q_moveit))
        trayectoria, t_plan = self.planificar(q_moveit)
        fila['t_plan_moveit'] = t_plan
        if trayectoria is not None:
            pts = trayectoria.joint_trajectory.points
            d = pts[-1].time_from_start
            fila['dur_moveit'] = d.sec + d.nanosec * 1e-9
            fila['n_moveit'] = len(pts)
            exito, t_total = self.ejecutar_trayectoria(trayectoria)
            fila['ok_moveit'] = exito
            fila['t_eje_moveit'] = t_total
            fila['err_moveit'] = math.dist(self.pose_herramienta(), punto)
        return fila


def _mm(v):
    return '--' if v is None else '%.3f' % (v * 1000)


def _q(q):
    return ('--' if q is None else
            '%.2f, %.2f, %.1f (%s)' % (math.degrees(q[0]), math.degrees(q[1]),
                                       q[2] * 1000, _rama(q)))


def informe(filas):
    L = []
    L.append('| punto | objetivo world (m) | q propia (deg, deg, mm) | '
             'q MoveIt (KDL) | max dq | err IK propia (mm) | '
             'err IK MoveIt (mm) | err final propia (mm) | '
             'err final MoveIt (mm) |')
    L.append('|---|---|---|---|---|---|---|---|---|')
    for f in filas:
        L.append('| %s | (%.3f, %.3f, %.3f) | %s | %s | %s | %s | %s | %s | %s |'
                 % (f['punto'], *f['objetivo'], _q(f.get('q_propia')),
                    _q(f.get('q_moveit')),
                    '--' if f.get('dq') is None else '%.2e' % f['dq'],
                    _mm(f.get('err_ik_propia')), _mm(f.get('err_ik_moveit')),
                    _mm(f.get('err_propia')), _mm(f.get('err_moveit'))))
    L.append('')
    L.append('| punto | t IK propia | t IK MoveIt | t plan MoveIt | '
             'duracion perfil propio | duracion trayectoria MoveIt | '
             't ejecucion propia | t ejecucion MoveIt |')
    L.append('|---|---|---|---|---|---|---|---|')
    for f in filas:
        us = lambda k: '--' if f.get(k) is None else '%.1f us' % (f[k] * 1e6)
        ms = lambda k: '--' if f.get(k) is None else '%.1f ms' % (f[k] * 1e3)
        seg = lambda k: '--' if f.get(k) is None else '%.2f s' % f[k]
        L.append('| %s | %s | %s | %s | %s | %s | %s | %s |'
                 % (f['punto'], us('t_ik_propia'), ms('t_ik_moveit'),
                    ms('t_plan_moveit'), seg('dur_propia'), seg('dur_moveit'),
                    seg('t_eje_propia'), seg('t_eje_moveit')))
    notas = [f for f in filas if f.get('motivo') or f.get('cod_moveit')]
    if notas:
        L.append('')
        for f in notas:
            L.append('- %s: %s' % (f['punto'], f.get('motivo') or
                                   'MoveIt no encontro IK (codigo %s)'
                                   % f.get('cod_moveit')))
    return '\n'.join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--salida', default='resultados/estacion2')
    args = ap.parse_args(argv if argv is not None else
                         rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    nodo = Comparador()
    filas = []
    try:
        for nombre, punto in PUNTOS_PRUEBA:
            nodo.get_logger().info('midiendo %s' % nombre)
            filas.append(nodo.medir(nombre, punto))
        nodo.volver_a_reposo()
    finally:
        texto = informe(filas)
        print('\n' + texto + '\n')
        os.makedirs(args.salida, exist_ok=True)
        ruta = os.path.join(args.salida, 'comparacion_moveit_vs_propia.md')
        with open(ruta, 'w') as fh:
            fh.write('# Comparacion MoveIt contra codigo propio\n\n')
            fh.write('Estacion 2 (TRIADA), %s. Generado por '
                     '`ros2 run station2_control comparar_moveit`.\n\n'
                     % time.strftime('%Y-%m-%d %H:%M'))
            fh.write(texto + '\n')
        print('escrito en', ruta)
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
