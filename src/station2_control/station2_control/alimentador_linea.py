"""Alimentador de prueba: hace de Estacion 1 fisica cuando la 1 es un mock.

    ros2 launch station2_description simulacion.launch.py gui:=false
    ros2 run station2_control vision
    ros2 run station2_control nodo_estacion
    ros2 run station2_control alimentador_linea
    ros2 launch line_bringup line.launch.py mock_stations:=station1,station3

Con la Estacion 1 como nodo de prueba nadie deja una pieza en el traspaso
1 -> 2, y la Estacion 2 no tendria nada que recoger. Este nodo escucha el
estado de la Estacion 1 y, cada vez que pasa a BUSY (el orquestador le acaba
de pedir una pieza):

  1. aparca la pieza del ciclo anterior, que ya entrego la Estacion 2;
  2. coloca la siguiente pieza de color sobre el soporte del traspaso 1 -> 2;
  3. le dice al nodo de la Estacion 2 que modelo acoplar.

Todo ocurre durante los segundos que tarda el mock de la Estacion 1, antes de
que el orquestador llame a la Estacion 2. Asi la prueba ejercita el contrato
real del orquestador (start, done, state, categoria) con la Estacion 2
completa: control, vision y entrega.
"""
import itertools

import rclpy
from line_interfaces.msg import StationState
from rcl_interfaces.msg import Parameter as ParametroMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

from station2_control.prueba_estacion import (APARCAMIENTO, Z_PIEZA,
                                              gz_mover)
from station2_control.punto_a_punto import PUNTOS

ORDEN = ('pieza_roja', 'pieza_verde', 'pieza_azul')


class Alimentador(Node):

    def __init__(self):
        super().__init__('station2_alimentador_prueba')
        self._ciclo = itertools.cycle(ORDEN)
        self._anterior = None
        self._estado = None
        self._param = self.create_client(
            SetParameters, '/station2_nodo_estacion/set_parameters')
        self.create_subscription(StationState, '/line/station1/state',
                                 self._al_estado, 10)
        self.get_logger().info('alimentador listo, esperando a la Estacion 1')

    def _al_estado(self, msg):
        previo, self._estado = self._estado, msg.state
        if msg.state == StationState.BUSY and previo != StationState.BUSY:
            self._alimentar()

    def _alimentar(self):
        pieza = next(self._ciclo)
        if self._anterior:
            gz_mover(self._anterior, *APARCAMIENTO[self._anterior])
        origen = PUNTOS['traspaso_12']
        gz_mover(pieza, origen[0], origen[1], Z_PIEZA)
        self._anterior = pieza

        p = ParametroMsg()
        p.name = 'pieza'
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                 string_value=pieza)
        if self._param.wait_for_service(timeout_sec=5.0):
            self._param.call_async(SetParameters.Request(parameters=[p]))
        self.get_logger().info('%s colocada en el traspaso 1-2' % pieza)


def main(args=None):
    rclpy.init(args=args)
    nodo = Alimentador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
