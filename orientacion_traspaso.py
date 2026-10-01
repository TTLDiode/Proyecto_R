"""Que orientacion tiene la pieza cuando llega al punto de traspaso.

La guia pide que la Estacion 1 entregue el componente "en pose conocida". Pose
incluye orientacion, y ahi hay una consecuencia de tener tres grados de
libertad que conviene tener medida antes de que la descubra la Estacion 2.

Con una cadena RRP la guinada de la herramienta no es libre: vale theta1+theta2
y queda determinada en cuanto se fija la posicion. La pieza se coge rigida, de
modo que su giro final es

    giro_final = giro_inicial + (guinada_en_traspaso - guinada_en_recogida)

La guinada en el traspaso es siempre la misma, porque el punto es fijo. La de
recogida no: depende de donde estuviera la pieza en la bandeja, y la guia dice
expresamente que ahi las posiciones no estan totalmente definidas.
"""
import math

import sys
sys.path.insert(0, '/home/santiago/E2_propuesta/src/station1_control')
from station1_control import cinematica as cin                     # noqa: E402

PUNTOS = {
    'centro de la bandeja': (-0.180, 0.100, 0.050),
    'esquina (-,-)':        (-0.230, 0.050, 0.050),
    'esquina (-,+)':        (-0.230, 0.150, 0.050),
    'esquina (+,-)':        (-0.130, 0.050, 0.050),
    'esquina (+,+)':        (-0.130, 0.150, 0.050),
}
TRASPASO = (0.250, 0.000, 0.060)


def main():
    q_t, ok, _ = cin.inversa(TRASPASO)
    assert ok
    guinada_traspaso = math.degrees(cin.guinada(q_t))
    print('Guinada de la herramienta en el traspaso: %.2f grados' % guinada_traspaso)
    print('Es siempre la misma, porque el punto de traspaso es fijo.\n')

    print('%-24s %12s %14s' % ('punto de recogida', 'guinada', 'giro de la pieza'))
    giros = []
    for nombre, punto in PUNTOS.items():
        q, ok, _ = cin.inversa(punto)
        if not ok:
            continue
        g = math.degrees(cin.guinada(q))
        giro = (guinada_traspaso - g + 180) % 360 - 180
        giros.append(giro)
        print('%-24s %9.2f deg %11.2f deg' % (nombre, g, giro))

    print('\nLa pieza llega girada entre %.1f y %.1f grados segun de donde se'
          % (min(giros), max(giros)))
    print('recoja, es decir una dispersion de %.1f grados.' % (max(giros) - min(giros)))
    print()
    print('Consecuencia para el contrato de interfaz: la Estacion 1 entrega la')
    print('pieza en una POSICION conocida y repetible, con error de 0.014 mm,')
    print('pero su ORIENTACION depende de donde estuviera en la bandeja. Con')
    print('tres grados de libertad no hay forma de corregirlo: la guinada no es')
    print('una variable libre, la fija la posicion.')
    print()
    print('Si la Estacion 2 necesita orientacion conocida hay tres salidas, y')
    print('conviene elegir una antes de la integracion:')
    print('  1. Que la pieza sea indiferente al giro, por ejemplo cuadrada.')
    print('  2. Que la Estacion 2 la mida con su camara, que ya lleva.')
    print('  3. Que el soporte del traspaso tenga un alojamiento que la oriente')
    print('     al depositarla, que es lo que se hace en una linea real.')


if __name__ == '__main__':
    main()
