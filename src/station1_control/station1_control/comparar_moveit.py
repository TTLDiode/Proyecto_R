"""Contraste entre MoveIt y el codigo propio sobre los puntos de tarea.

    ros2 run station1_control comparar_moveit
    ros2 run station1_control comparar_moveit --salida ~/E2_propuesta/resultados

Es el entregable que pide el enunciado de la Entrega 2: "usado para planear y
ejecutar movimientos punto a punto, contrastando los resultados de MoveIt con
el codigo propio". Para cada punto de tarea mide, por las dos vias:

  - la solucion articular, y cuanto se separan entre si
  - el error de la solucion contra la cinematica directa, que dice cual de las
    dos resuelve realmente la ecuacion
  - el tiempo que tarda en obtenerla
  - el tiempo de planificacion, cuando lo hay
  - el error de la pose finalmente alcanzada, leida del arbol TF del simulador
  - la duracion de la ejecucion

Las dos vias parten siempre del mismo estado, al que se vuelve entre medida y
medida, porque el tiempo de un movimiento depende de donde empiece.

Una advertencia sobre como leer la tabla: que las dos soluciones coincidan no
es una casualidad afortunada, es lo que tiene que pasar. La inversa propia es
cerrada y exacta; KDL es iterativo y converge a la misma solucion cuando
arranca cerca de ella. Lo interesante no es la coincidencia sino el coste: la
inversa cerrada da la respuesta en microsegundos y siempre, y la numerica tarda
milisegundos y puede no converger.
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

from station1_control import cinematica as cin
from station1_control import dinamica as din
from station1_control import trayectoria as tra
from station1_control.punto_a_punto import PUNTOS, ControlPuntoAPunto

# Estado comun de partida. Ni la bandeja ni el traspaso, para que ninguna de
# las dos medidas salga favorecida por empezar cerca de su objetivo.
REPOSO = (math.radians(90.0), math.radians(90.0), 0.0)


class Comparador(ControlPuntoAPunto):

    def __init__(self):
        super().__init__()
        self._ik = self.create_client(GetPositionIK, '/compute_ik')
        # La accion se llama move_action; move_group es el nodo que la sirve.
        self._plan = ActionClient(self, MoveGroup, '/move_action')
        self._ejecutar = ActionClient(self, ExecuteTrajectory,
                                      '/execute_trajectory')

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
            raise RuntimeError('move_group no ofrece /compute_ik')

        pet = GetPositionIK.Request()
        pet.ik_request = PositionIKRequest()
        pet.ik_request.group_name = 'brazo'
        pet.ik_request.ik_link_name = 'station1/tool0'
        pet.ik_request.robot_state = self.estado_actual()
        pet.ik_request.avoid_collisions = True
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.pose = Pose()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = punto
        # La orientacion se rellena por completitud; con position_only_ik el
        # solucionador la ignora, que es lo correcto para tres grados de
        # libertad.
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
        """Solo planifica. Devuelve (trayectoria, segundos de planificacion).

        Se separa de la ejecucion a proposito. El campo planning_time que
        devuelve MoveIt llega a cero en problemas tan sencillos como este, de
        modo que el tiempo se mide con reloj propio, de extremo a extremo de la
        peticion, que es ademas lo que costaria en una aplicacion real.
        """
        if not self._plan.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('no responde la accion /move_action')

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
            raise RuntimeError('no responde la accion /execute_trajectory')
        meta = ExecuteTrajectory.Goal()
        meta.trajectory = trayectoria
        t0 = time.perf_counter()
        manejador = self._esperar(self._ejecutar.send_goal_async(meta), 30.0)
        if manejador is None or not manejador.accepted:
            return False, 0.0
        r = self._esperar(manejador.get_result_async(), plazo)
        dt = time.perf_counter() - t0
        return (r is not None and r.result.error_code.val == 1), dt

    def par_de_la_trayectoria(self, trayectoria):
        """Par maximo que exige una trayectoria de MoveIt, con el modelo propio.

        MoveIt reparametriza el tiempo con los limites por articulacion de
        joint_limits.yaml, que son independientes entre si. El modelo dinamico
        de dinamica.py dice cuanto par exige de verdad, acoplamiento incluido.
        Comparar las dos cifras es la parte interesante del contraste.
        """
        pts = trayectoria.joint_trajectory.points
        nombres = list(trayectoria.joint_trajectory.joint_names)
        idx = [nombres.index(j) for j in cin.ARTICULACIONES]
        peor = 0.0
        for p in pts:
            if not p.accelerations:
                continue
            q = [p.positions[i] for i in idx]
            qd = [p.velocities[i] for i in idx]
            qdd = [p.accelerations[i] for i in idx]
            t1, t2 = din.pares(q, qd, qdd)
            peor = max(peor, abs(t1), abs(t2))
        dur = pts[-1].time_from_start
        return peor, dur.sec + dur.nanosec*1e-9

    # ---------------------------------------------------------------
    def volver_a_reposo(self):
        self.mover_a_articular(REPOSO)
        time.sleep(0.3)
        for _ in range(10):
            self._girar(0.05)

    def medir(self, nombre, punto):
        fila = {'punto': nombre, 'objetivo': punto}

        # --- via propia ---
        self.volver_a_reposo()
        t0 = time.perf_counter()
        q_propia, ok, motivo = cin.inversa(punto)
        fila['t_ik_propia'] = time.perf_counter() - t0
        fila['q_propia'] = q_propia
        fila['err_ik_propia'] = cin.error_cierre(punto, q_propia) if ok else None
        if ok:
            q_ini = self.esperar_estado()
            T = tra.duracion(q_ini, q_propia)
            fila['par_propia'] = din.par_maximo(q_ini, q_propia, T)
            fila['dur_propia'] = T
            exito, T, t_real = self.mover_a_articular(q_propia)
            alcanzado = self.pose_herramienta()
            fila['ok_propia'] = exito
            fila['t_eje_propia'] = t_real
            fila['err_propia'] = math.dist(alcanzado, punto)

        # --- via MoveIt ---
        self.volver_a_reposo()
        q_moveit, t_ik, cod = self.ik_moveit(punto)
        fila['t_ik_moveit'] = t_ik
        fila['q_moveit'] = q_moveit
        if q_moveit is not None:
            fila['err_ik_moveit'] = cin.error_cierre(punto, q_moveit)
            fila['dq'] = max(abs(a-b) for a, b in zip(q_propia, q_moveit))
            trayectoria, t_plan = self.planificar(q_moveit)
            fila['t_plan_moveit'] = t_plan
            if trayectoria is not None:
                par, dur = self.par_de_la_trayectoria(trayectoria)
                fila['par_moveit'] = par
                fila['dur_moveit'] = dur
                exito, t_total = self.ejecutar_trayectoria(trayectoria)
                alcanzado = self.pose_herramienta()
                fila['ok_moveit'] = exito
                fila['t_eje_moveit'] = t_total
                fila['err_moveit'] = math.dist(alcanzado, punto)
        else:
            fila['cod_moveit'] = cod
        return fila


def _mm(v):
    return '--' if v is None else '%.3f' % (v*1000)


def informe(filas):
    L = []
    L.append('| punto | q propia (deg, deg, mm) | q MoveIt | max |dq| | '
             'err IK propia | err IK MoveIt | err final propia | err final MoveIt |')
    L.append('|---|---|---|---|---|---|---|---|')
    for f in filas:
        qp = f.get('q_propia'); qm = f.get('q_moveit')
        fmt = lambda q: ('--' if q is None else
                         '%.2f, %.2f, %.1f' % (math.degrees(q[0]),
                                               math.degrees(q[1]), q[2]*1000))
        L.append('| %s | %s | %s | %s | %s mm | %s mm | %s mm | %s mm |'
                 % (f['punto'], fmt(qp), fmt(qm),
                    ('--' if f.get('dq') is None else '%.2e' % f['dq']),
                    _mm(f.get('err_ik_propia')), _mm(f.get('err_ik_moveit')),
                    _mm(f.get('err_propia')), _mm(f.get('err_moveit'))))
    L.append('')
    L.append('| punto | t IK propia | t IK MoveIt | t plan MoveIt | '
             't ejecucion propia | t ejecucion MoveIt |')
    L.append('|---|---|---|---|---|---|')
    for f in filas:
        us = lambda k: ('--' if f.get(k) is None else '%.1f us' % (f[k]*1e6))
        seg = lambda k: ('--' if f.get(k) is None else '%.3f s' % f[k])
        L.append('| %s | %s | %s | %s | %s | %s |'
                 % (f['punto'], us('t_ik_propia'), seg('t_ik_moveit'),
                    seg('t_plan_moveit'), seg('t_eje_propia'),
                    seg('t_eje_moveit')))
    L.append('')
    L.append('Par maximo que exige cada trayectoria, evaluado con el mismo '
             'modelo dinamico. El presupuesto es %.3f N.m, el par que deja '
             'pasar el TB6612FNG.' % din.TAU_MAX)
    L.append('')
    L.append('| punto | duracion propia | par propia | duracion MoveIt | '
             'par MoveIt | par MoveIt / presupuesto |')
    L.append('|---|---|---|---|---|---|')
    for f in filas:
        nm = lambda k: ('--' if f.get(k) is None else '%.3f N.m' % f[k])
        seg = lambda k: ('--' if f.get(k) is None else '%.2f s' % f[k])
        rel = ('--' if f.get('par_moveit') is None
               else '%.0f %%' % (100*f['par_moveit']/din.TAU_MAX))
        L.append('| %s | %s | %s | %s | %s | %s |'
                 % (f['punto'], seg('dur_propia'), nm('par_propia'),
                    seg('dur_moveit'), nm('par_moveit'), rel))
    return '\n'.join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--salida', default=os.path.expanduser('~/E2_propuesta/resultados'))
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    nodo = Comparador()
    filas = []
    try:
        for nombre in ('bandeja', 'esquina1', 'esquina2', 'esquina3',
                       'esquina4', 'traspaso'):
            nodo.get_logger().info('midiendo %s' % nombre)
            filas.append(nodo.medir(nombre, PUNTOS[nombre]))
    finally:
        texto = informe(filas)
        print('\n' + texto + '\n')
        os.makedirs(args.salida, exist_ok=True)
        ruta = os.path.join(args.salida, 'comparacion_moveit_vs_propia.md')
        with open(ruta, 'w') as fh:
            fh.write('# Comparacion MoveIt contra codigo propio\n\n')
            fh.write('Estacion 1, %s\n\n' % time.strftime('%Y-%m-%d %H:%M'))
            fh.write(texto + '\n')
        print('escrito en', ruta)
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
