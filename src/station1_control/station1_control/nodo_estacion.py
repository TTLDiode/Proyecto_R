"""Nodo de la Estacion 1: ejecuta el ciclo de alimentacion y singulacion y habla
el contrato de interfaz de la linea.

    ros2 run station1_control nodo_estacion

La estacion es una caja negra para las demas, segun ADR-004. Lo unico que debe
cumplir es el protocolo de docs/03-contrato-de-interfaz.md:

    /line/station1/start   servicio StartTask   entrada, arranca un ciclo
    /line/station1/done    topico HandoffEvent  salida, una vez por ciclo
    /line/station1/state   topico StationState  salida, latido a 2 Hz

El ciclo que ejecuta es el que pide la guia para esta estacion: tomar una pieza
de la bandeja de entrada, cuyas posiciones no estan totalmente definidas, y
entregarla en pose conocida en el punto fijo de traspaso a la Estacion 2.

    reposo -> sobre la pieza -> descenso -> cerrar mordaza -> ascenso
           -> sobre el traspaso -> descenso -> abrir mordaza -> ascenso -> reposo

Por que la aproximacion es siempre vertical. La pieza se coge y se suelta
bajando y subiendo en linea recta sobre el punto, nunca en diagonal. Con una
cadena RRP eso sale gratis: el descenso es la prismatica sola, con las dos
rotacionales quietas, de modo que no hay forma de que el movimiento de bajada
arrastre la pieza lateralmente. Es la ventaja practica de la configuracion y es
la razon de que la eleccion RRP se defienda para una estacion de pick and place.

El estado se publica siempre, incluso mientras la estacion esta parada, porque
el orquestador distingue "ocupada" de "caida" por la ausencia del latido y no
por el silencio en /done.
"""
import math
import threading
import time

import rclpy
from line_interfaces.msg import HandoffEvent, StationState
from line_interfaces.srv import StartTask
from std_msgs.msg import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from station1_control import cinematica as cin
from station1_control.punto_a_punto import PUNTOS, ControlPuntoAPunto

# Altura de aproximacion sobre el punto de trabajo.
#
# Es la cota que mas pesa en el tiempo de ciclo, porque se recorre cuatro veces
# y siempre con la prismatica sola, que es la articulacion lenta: a 0.0187 m/s
# de pico, cada 10 mm de aproximacion cuestan un segundo del ciclo.
#
# 20 mm es lo minimo defendible: la pieza mide 12 mm de alto y quedan 8 mm de
# holgura entre su cara superior y la mordaza al desplazarse en horizontal.
APROXIMACION = 0.020

REPOSO = (math.radians(90.0), math.radians(90.0), 0.0)


class NodoEstacion1(ControlPuntoAPunto):

    def __init__(self):
        super().__init__()
        # Este nodo lo gira un MultiThreadedExecutor y el ciclo corre en un
        # hilo aparte, de modo que las esperas de la clase base no deben
        # girarlo por su cuenta.
        self._girado_fuera = True
        grupo = MutuallyExclusiveCallbackGroup()

        self._estado_linea = StationState.IDLE
        self._detalle = 'recien arrancada'
        self._ocupada = threading.Lock()
        self._contador = 0
        self._tramos = []

        # Acople cinematico de la pieza, ADR-002. El agarre no lo hace el
        # rozamiento de la mordaza sino esta union, que Gazebo crea y deshace
        # cuando se le avisa. El servo se mueve igualmente, porque es lo que
        # se ve y lo que habria en la maquina real, pero quien sujeta es esto.
        self._pub_acoplar = self.create_publisher(
            Empty, '/station1/pieza/attach', 10)
        self._pub_soltar = self.create_publisher(
            Empty, '/station1/pieza/detach', 10)

        self._pub_estado = self.create_publisher(
            StationState, '/line/station1/state', 10)
        self._pub_fin = self.create_publisher(
            HandoffEvent, '/line/station1/done', 10)
        self.create_timer(0.5, self._latido)
        self._servicio = self.create_service(
            StartTask, '/line/station1/start', self._al_pedir_inicio,
            callback_group=grupo)

        self.get_logger().info('Estacion 1 lista, esperando en '
                               '/line/station1/start')

    # ------------------------------------------------------------------
    def _latido(self):
        msg = StationState()
        msg.state = self._estado_linea
        msg.station_id = 'station1'
        msg.detail = self._detalle
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_estado.publish(msg)

    def _al_pedir_inicio(self, peticion, respuesta):
        """Arranca un ciclo. Rechaza si ya hay uno en marcha.

        El rechazo no es un error: el contrato dice que la estacion responde
        accepted=false si esta ocupada, y el orquestador lo reintenta. Lo que no
        puede pasar es que dos ciclos se solapen sobre la misma maquina.
        """
        if not self._ocupada.acquire(blocking=False):
            respuesta.accepted = False
            respuesta.message = 'ciclo en curso'
            return respuesta

        try:
            origen = self._origen_de(peticion.input)
            hilo = threading.Thread(target=self._ciclo, args=(peticion.input,
                                                              origen),
                                    daemon=True)
            hilo.start()
            respuesta.accepted = True
            respuesta.message = 'ciclo iniciado'
        except Exception as exc:                       # noqa: BLE001
            self._ocupada.release()
            respuesta.accepted = False
            respuesta.message = str(exc)
        return respuesta

    def _origen_de(self, evento):
        """De donde recoger la pieza.

        La guia dice que las posiciones en la bandeja no estan totalmente
        definidas, de modo que quien pide el ciclo puede indicar donde esta la
        pieza. Si no lo indica, se usa el centro de la bandeja: es el
        comportamiento util para probar la estacion sola.
        """
        p = evento.pose.position
        if abs(p.x) < 1e-9 and abs(p.y) < 1e-9 and abs(p.z) < 1e-9:
            return PUNTOS['bandeja']
        return (p.x, p.y, p.z)

    # ------------------------------------------------------------------
    def _ciclo(self, evento, origen):
        destino = PUNTOS['traspaso']
        t0 = time.time()
        self._tramos = []
        self._estado_linea = StationState.BUSY
        self._detalle = 'recogiendo'
        try:
            # No se pasa por reposo antes de recoger: el ciclo anterior ya deja
            # el brazo ahi y, si no, el movimiento a la aproximacion es
            # igualmente valido desde cualquier pose alcanzable. Hacerlo
            # costaba 4.6 s de los 25.8 que medimos al principio.
            self._coger(origen)
            self._detalle = 'entregando'
            self._soltar(destino)
            # El retroceso a reposo si se conserva: deja el brazo fuera del
            # punto de traspaso, que es donde la Estacion 2 va a meter el suyo.
            self._ir(REPOSO, 'retroceso a reposo')

            duracion = time.time() - t0
            self.get_logger().info('reparto del ciclo: %s' % ', '.join(
                '%s %.1f s' % (n, d) for n, d in self._tramos))
            self._contador += 1
            self._estado_linea = StationState.DONE
            self._detalle = 'ciclo %d en %.1f s' % (self._contador, duracion)
            self.get_logger().info('ciclo %d completo en %.1f s (limite 30 s)'
                                   % (self._contador, duracion))
            self._anunciar_fin(evento, destino)
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
            raise RuntimeError('el brazo no alcanzo la pose articular pedida')
        if etiqueta:
            self._tramos.append((etiqueta, time.time() - t0))

    def _ir_a_punto(self, punto, etiqueta=''):
        q, ok, motivo = cin.inversa(punto)
        if not ok:
            raise RuntimeError('punto inalcanzable: %s' % motivo)
        self._ir(q, etiqueta)

    def _coger(self, punto):
        arriba = (punto[0], punto[1], punto[2] + APROXIMACION)
        self.abrir_mordaza()
        self._ir_a_punto(arriba, 'aproximacion a la pieza')
        self._ir_a_punto(punto, 'descenso sobre la pieza')
        self.cerrar_mordaza()
        self._acoplar()
        self._ir_a_punto(arriba, 'ascenso con la pieza')

    def _soltar(self, punto):
        arriba = (punto[0], punto[1], punto[2] + APROXIMACION)
        self._ir_a_punto(arriba, 'traslado al traspaso')
        self._ir_a_punto(punto, 'descenso en el traspaso')
        # Primero se deshace el acople y despues se abre la mordaza. Al reves,
        # la pieza acompanaria el giro del servo antes de quedar libre.
        self._soltar_pieza()
        self.abrir_mordaza()
        self._ir_a_punto(arriba, 'ascenso sin la pieza')

    def _acoplar(self):
        self._pub_acoplar.publish(Empty())
        time.sleep(0.4)          # el acople se materializa en el siguiente paso
        self.get_logger().info('pieza acoplada a la mordaza')

    def _soltar_pieza(self):
        self._pub_soltar.publish(Empty())
        time.sleep(0.4)
        self.get_logger().info('pieza liberada')

    def _anunciar_fin(self, evento, destino):
        msg = HandoffEvent()
        msg.part_id = evento.part_id if evento.part_id else self._contador
        msg.category = evento.category          # la E1 no clasifica
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = destino
        msg.pose.orientation.w = 1.0
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_fin.publish(msg)


def main(argv=None):
    rclpy.init()
    nodo = NodoEstacion1()
    ejecutor = MultiThreadedExecutor()
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
