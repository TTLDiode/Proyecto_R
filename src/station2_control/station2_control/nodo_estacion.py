"""Nodo de la Estacion 2: recibe la pieza, la inspecciona, la clasifica y la
entrega a la Estacion 3, hablando el contrato de interfaz de la linea.

    ros2 run station2_control nodo_estacion
    ros2 run station2_control nodo_estacion --ros-args -p pieza:=pieza_verde

Adaptado de station1_control/nodo_estacion.py. Contrato (ADR-004):

    /line/station2/start   servicio StartTask   entrada; rechaza si esta ocupada
    /line/station2/done    topico HandoffEvent  una vez por ciclo, con categoria
    /line/station2/state   topico StationState  latido a 2 Hz

Ciclo:

    traspaso 1-2: aproximar -> bajar -> cerrar -> acoplar -> subir
    puesto de inspeccion: bajar -> soltar -> subir
    camara sobre la pieza -> /station2/clasificar -> categoria
    puesto de inspeccion: bajar -> cerrar -> acoplar -> subir
    traspaso 2-3: bajar -> desacoplar -> abrir -> subir -> reposo
    publicar done con la categoria y la pose de entrega

Por que un puesto de inspeccion. La camara va bajo el eslabon 2, 70 mm por
detras del vastago, de modo que no puede ver la pieza mientras la mordaza la
sujeta, y la lente no llega encima del traspaso 1-2: ese punto esta a 250 mm
de la base y la lente alcanza como mucho 230.6 mm (eslabon 1 mas 70.6 mm).
El puesto esta a 200 mm de la base, en +y, donde llegan tanto la lente como
la mordaza, y a la misma altura que los soportes de traspaso.

Las aproximaciones son verticales: en una RRP el descenso es la prismatica
sola, con las rotacionales quietas, y la pieza no se arrastra de lado.
"""
import math
import threading
import time

import rclpy
from line_interfaces.msg import HandoffEvent, StationState
from line_interfaces.srv import StartTask
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from station2_control import cinematica as cin
from station2_control.punto_a_punto import (DEDO_CERRADO, PUNTOS,
                                            ControlPuntoAPunto, cierre_para)

# Giro de la pieza en el traspaso 1-2 (yaw de handoff_1_2 en line_poses.yaml).
GIRO_PIEZA_12 = 0.0

# Altura de aproximacion sobre cada punto de trabajo. La pieza mide 12 mm y
# quedan 8 mm de holgura al desplazarse en horizontal, como en la Estacion 1.
APROXIMACION = 0.020

# Bajada extra para coger y soltar. Los puntos de traspaso (z = 0.060) son la
# altura de referencia de la linea; la pieza, de 12 mm, esta apoyada en el
# soporte con su centro a z = 0.048 (de 0.042 a 0.054). tool0 son las puntas
# de los dedos, que suben 26 mm desde ahi: para que los dedos abracen la
# pieza, las puntas bajan hasta 2 mm sobre el soporte (z = 0.044, d3 = 76 mm).
# El acople de Gazebo fija la pieza en la pose que tiene al acoplar; con la
# bajada a 0.060 la pieza quedaba colgando 6 mm por debajo de los dedos.
AGARRE = 0.016

# Espera para que la camara entregue una imagen tomada ya en la pose de
# inspeccion: la camara simulada da unas 2-5 imagenes por segundo.
ASENTAMIENTO_CAMARA = 1.5

# Reposo: brazo apuntando a +y, fuera de los dos traspasos.
REPOSO = (math.radians(90.0), 0.0, 0.0)


class NodoEstacion2(ControlPuntoAPunto):

    def __init__(self):
        super().__init__('station2_nodo_estacion')
        self._girado_fuera = True
        grupo = MutuallyExclusiveCallbackGroup()

        # Modelo de Gazebo que se acopla a la mordaza (ADR-002). En la linea
        # es "pieza", la que trae la Estacion 1; en las pruebas, una de las
        # piezas de colores del paquete.
        self.declare_parameter('pieza', 'pieza')

        self._estado_linea = StationState.IDLE
        self._detalle = 'recien arrancada'
        self._ocupada = threading.Lock()
        self._contador = 0
        self._tramos = []
        self.ultimo = {}

        self._pub_estado = self.create_publisher(
            StationState, '/line/station2/state', 10)
        self._pub_fin = self.create_publisher(
            HandoffEvent, '/line/station2/done', 10)
        self._vision = self.create_client(Trigger, '/station2/clasificar')
        self.create_timer(0.5, self._latido)
        self._servicio = self.create_service(
            StartTask, '/line/station2/start', self._al_pedir_inicio,
            callback_group=grupo)
        self._acople = {}
        self._cierre = DEDO_CERRADO
        self.get_logger().info('Estacion 2 lista, esperando en '
                               '/line/station2/start')

    # ------------------------------------------------------------------
    def _latido(self):
        msg = StationState()
        msg.state = self._estado_linea
        msg.station_id = 'station2'
        msg.detail = self._detalle
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_estado.publish(msg)

    def _al_pedir_inicio(self, peticion, respuesta):
        if not self._ocupada.acquire(blocking=False):
            respuesta.accepted = False
            respuesta.message = 'ciclo en curso'
            return respuesta
        hilo = threading.Thread(target=self._ciclo, args=(peticion.input,),
                                daemon=True)
        hilo.start()
        respuesta.accepted = True
        respuesta.message = 'ciclo iniciado'
        return respuesta

    # ------------------------------------------------------------------
    def _ciclo(self, evento):
        t0 = time.time()
        self._tramos = []
        self._estado_linea = StationState.BUSY
        try:
            pieza = self.get_parameter('pieza').value
            # La pieza llega con giro 0 (handoff_1_2) y la pinza esta girada
            # theta1 + theta2 en el traspaso 1-2. Ese giro relativo se
            # conserva durante todo el ciclo (la pieza gira con la pinza), asi
            # que el cierre se calcula una vez.
            q12, _, _ = cin.inversa(cin.a_base(PUNTOS['traspaso_12']), 'arriba')
            self._cierre = cierre_para(GIRO_PIEZA_12 - cin.guinada(q12))
            self._detalle = 'recogiendo en el traspaso 1-2'
            self._coger(PUNTOS['traspaso_12'], pieza, rama='arriba')
            err_12 = self._error_herramienta_pendiente

            self._detalle = 'inspeccionando'
            self._soltar(PUNTOS['inspeccion'], pieza)
            categoria = self._inspeccionar(PUNTOS['inspeccion'])
            self._coger(PUNTOS['inspeccion'], pieza)

            self._detalle = 'entregando en el traspaso 2-3'
            self._soltar(PUNTOS['traspaso_23'], pieza, rama='abajo')
            err_23 = self._error_herramienta_pendiente
            self._ir(REPOSO, 'retroceso a reposo')

            duracion = time.time() - t0
            self._contador += 1
            self.ultimo = {'categoria': categoria, 'duracion': duracion,
                           'error_12': err_12, 'error_23': err_23}
            self.get_logger().info('reparto del ciclo: %s' % ', '.join(
                '%s %.1f s' % (n, d) for n, d in self._tramos))
            self.get_logger().info(
                'ciclo %d en %.1f s: categoria %s, error de tool0 %.2f mm en '
                'el traspaso 1-2 y %.2f mm en el 2-3 (limite 20 mm)'
                % (self._contador, duracion, categoria, err_12*1000,
                   err_23*1000))
            self._estado_linea = StationState.DONE
            self._detalle = 'ciclo %d: categoria %s' % (self._contador,
                                                        categoria)
            self._anunciar_fin(evento, categoria)
            self._estado_linea = StationState.IDLE
        except Exception as exc:                       # noqa: BLE001
            self._estado_linea = StationState.ERROR
            self._detalle = str(exc)
            self.get_logger().error('ciclo abortado: %s' % exc)
        finally:
            self._ocupada.release()

    def _ir(self, q, etiqueta=''):
        t0 = time.time()
        ok, _, _ = self.mover_a_articular(q)
        if not ok:
            raise RuntimeError('el brazo no alcanzo la pose articular pedida '
                               '(%s)' % etiqueta)
        if etiqueta:
            self._tramos.append((etiqueta, time.time() - t0))

    def _ir_a_punto(self, punto, etiqueta='', rama='auto'):
        q, ok, motivo = cin.inversa(cin.a_base(punto), rama=rama)
        if not ok:
            raise RuntimeError('punto %s inalcanzable: %s' % (punto, motivo))
        self._ir(q, etiqueta)

    def _medir(self, punto):
        """Error de tool0 contra el punto pedido, leido del arbol TF."""
        alcanzado = self.pose_herramienta()
        return math.dist(alcanzado, punto)

    def _coger(self, punto, pieza, rama='auto'):
        arriba = (punto[0], punto[1], punto[2] + APROXIMACION)
        bajo = (punto[0], punto[1], punto[2] - AGARRE)
        self.abrir_mordaza()
        self._ir_a_punto(arriba, 'aproximacion', rama)
        self._ir_a_punto(bajo, 'descenso a coger', rama)
        self._error_herramienta_pendiente = self._medir(bajo)
        self.cerrar_mordaza(self._cierre)
        self._acoplar(pieza)
        self._ir_a_punto(arriba, 'ascenso con la pieza', rama)

    def _soltar(self, punto, pieza, rama='auto'):
        arriba = (punto[0], punto[1], punto[2] + APROXIMACION)
        bajo = (punto[0], punto[1], punto[2] - AGARRE)
        self._ir_a_punto(arriba, 'traslado', rama)
        self._ir_a_punto(bajo, 'descenso a soltar', rama)
        self._error_herramienta_pendiente = self._medir(bajo)
        # Primero se suelta el acople y despues se abre la mordaza.
        self._desacoplar(pieza)
        self.abrir_mordaza()
        self._ir_a_punto(arriba, 'ascenso sin la pieza', rama)

    def _inspeccionar(self, punto):
        """Pone la lente sobre la pieza y pide la categoria a la vision."""
        q, ok, motivo = cin.inversa_camara(cin.a_base(punto)[:2])
        if not ok:
            raise RuntimeError('la camara no llega al puesto: %s' % motivo)
        self._ir(q, 'camara sobre la pieza')
        time.sleep(ASENTAMIENTO_CAMARA)
        if not self._vision.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('el nodo de vision no responde')
        r = self._esperar(self._vision.call_async(Trigger.Request()), 10.0)
        if r is None or not r.success:
            raise RuntimeError('la vision no reconocio la pieza: %s'
                               % (r.message if r else 'sin respuesta'))
        self.get_logger().info('categoria asignada por la vision: %s'
                               % r.message)
        return r.message

    def _publicador(self, pieza, accion):
        clave = (pieza, accion)
        if clave not in self._acople:
            self._acople[clave] = self.create_publisher(
                Empty, '/station2/%s/%s' % (pieza, accion), 10)
        return self._acople[clave]

    def _acoplar(self, pieza):
        self._publicador(pieza, 'attach').publish(Empty())
        time.sleep(0.4)
        self.get_logger().info('%s acoplada a la mordaza' % pieza)

    def _desacoplar(self, pieza):
        self._publicador(pieza, 'detach').publish(Empty())
        time.sleep(0.4)
        self.get_logger().info('%s liberada' % pieza)

    def _anunciar_fin(self, evento, categoria):
        msg = HandoffEvent()
        msg.part_id = evento.part_id if evento.part_id else self._contador
        msg.category = categoria
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = \
            PUNTOS['traspaso_23']
        msg.pose.orientation.w = 1.0
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_fin.publish(msg)


def main(argv=None):
    rclpy.init()
    nodo = NodoEstacion2()
    ejecutor = MultiThreadedExecutor()
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
