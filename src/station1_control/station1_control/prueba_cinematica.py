"""Prueba automatizada de ida y vuelta de la cinematica de la Estacion 1.

    ros2 run station1_control prueba_cinematica
    python3 -m station1_control.prueba_cinematica --barrido 0.001

Es el equivalente en Python de matlab/station1/e1_prueba_cinematica.m y da los
mismos numeros. Existen las dos versiones porque cada una responde a una cosa
distinta: MATLAB es lo que pide el enunciado para verificar la cinematica, y
esta corre sin licencia, dentro del paquete de ROS y como prueba de pytest, de
modo que una modificacion del codigo que rompa la cinematica se detecta al
construir el espacio de trabajo y no el dia de la sustentacion.

Comprueba tres cosas:
  1. Que directa e inversa cierran sobre todo el espacio alcanzable.
  2. Que los puntos imposibles se rechazan, y por el motivo correcto.
  3. Que los seis puntos de tarea salen con los valores documentados.

No basta con probar la inversa contra puntos conocidos: hay que barrer el
volumen, porque los fallos de una inversa cerrada aparecen en los bordes, no
en el centro.
"""
import argparse
import math
import sys

from station1_control import cinematica as cin

IMPOSIBLES = (
    ('mas alla del alcance',    ( 0.340,  0.000, 0.080), 'radio fuera del alcance'),
    ('dentro del radio muerto', ( 0.050,  0.000, 0.080), 'fuera de los limites de articulacion'),
    ('demasiado alto',          ( 0.250,  0.000, 0.150), 'altura fuera de la carrera'),
    ('demasiado bajo',          ( 0.250,  0.000, 0.020), 'altura fuera de la carrera'),
    ('detras, a -60 grados',    ( 0.150, -0.260, 0.080), 'fuera de los limites de articulacion'),
)

TAREA = (
    ('Centro de la bandeja',  (-0.180, 0.100, 0.050)),
    ('Esquina (-,-)',         (-0.230, 0.050, 0.050)),
    ('Esquina (-,+)',         (-0.230, 0.150, 0.050)),
    ('Esquina (+,-)',         (-0.130, 0.050, 0.050)),
    ('Esquina (+,+)',         (-0.130, 0.150, 0.050)),
    ('Traspaso a Estacion 2', ( 0.250, 0.000, 0.060)),
)

TOLERANCIA = 1e-9       # metros; el cierre real es del orden de 1e-16


def _rango(a, b, paso):
    n = int(math.floor((b - a) / paso + 1e-9)) + 1
    return (a + i * paso for i in range(n))


def barrido(paso_r=0.002, paso_ang=math.radians(2), paso_z=0.006):
    """Recorre el espacio alcanzable y devuelve el resumen del cierre."""
    peor = 0.0
    n_ok = n_no = arriba = 0
    for r in _rango(cin.R_MIN, cin.R_MAX, paso_r):
        for ang in _rango(cin.T1_MIN, cin.T1_MAX, paso_ang):
            for z in _rango(cin.Z0 - cin.D3_MAX, cin.Z0, paso_z):
                objetivo = (r*math.cos(ang), r*math.sin(ang), z)
                q, ok, _ = cin.inversa(objetivo)
                if ok:
                    n_ok += 1
                    arriba += (q[1] > 0)
                    peor = max(peor, cin.error_cierre(objetivo, q))
                else:
                    n_no += 1
    return {'ensayadas': n_ok + n_no, 'resueltas': n_ok, 'peor': peor,
            'arriba': arriba}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--barrido', type=float, default=0.002,
                    help='paso radial del barrido, en metros')
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    print('\n== Cierre directa-inversa sobre el espacio alcanzable ==')
    r = barrido(paso_r=args.barrido)
    print('  poses ensayadas        %d' % r['ensayadas'])
    print('  resueltas              %d  (%.1f %%)'
          % (r['resueltas'], 100*r['resueltas']/r['ensayadas']))
    print('  error maximo de cierre %.2e m' % r['peor'])
    print('  codo arriba            %.1f %%'
          % (100*r['arriba']/r['resueltas']))

    print('\n== Rechazo de puntos imposibles ==')
    for nombre, punto, esperado in IMPOSIBLES:
        _, ok, motivo = cin.inversa(punto)
        print('  %-26s %-9s %s' % (nombre, 'RESUELVE' if ok else 'rechaza', motivo))

    print('\n== Puntos de tarea ==')
    print('  %-24s %8s %8s %9s  %s' % ('punto', 'theta1', 'theta2', 'd3 (mm)', 'codo'))
    for nombre, punto in TAREA:
        q, ok, _ = cin.inversa(punto)
        if not ok:
            print('  %-24s  NO ALCANZABLE' % nombre)
            continue
        print('  %-24s %8.1f %8.1f %9.1f  %s'
              % (nombre, math.degrees(q[0]), math.degrees(q[1]), q[2]*1000,
                 'arriba' if q[1] > 0 else 'abajo'))
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
