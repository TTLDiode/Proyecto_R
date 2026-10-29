"""Prueba automatizada de ida y vuelta de la cinematica de la Estacion 2.

    ros2 run station2_control prueba_cinematica
    python3 -m station2_control.prueba_cinematica

Adaptada de station1_control/prueba_cinematica.py y equivalente a
matlab/station2/e2_prueba_cinematica.m. Comprueba:

  1. Ida y vuelta en todo el espacio articular: para cada q de una rejilla
     dentro de los limites, directa(q) da un punto y la inversa de ese punto,
     en la misma rama de codo, debe devolver el mismo q y cerrar sobre el
     punto. Se barre el espacio articular y no el cartesiano porque asi cada
     punto ensayado es alcanzable por construccion.
  2. Que los puntos imposibles se rechazan, y por el motivo correcto.
  3. Que los puntos de tarea (traspasos, puesto de inspeccion y la camara
     sobre el puesto) son alcanzables con los valores documentados.
"""
import argparse
import math

from station2_control import cinematica as cin

IMPOSIBLES = (
    ('mas alla del alcance',   (0.320, 0.000, 0.080), 'radio fuera del alcance'),
    ('dentro del radio muerto', (0.050, 0.000, 0.080),
     'fuera de los limites de articulacion'),
    ('demasiado alto',         (0.200, 0.000, 0.130), 'altura fuera de la carrera'),
    ('demasiado bajo',         (0.200, 0.000, 0.030), 'altura fuera de la carrera'),
    ('detras, a -80 grados',   (0.040, -0.200, 0.080),
     'fuera de los limites de articulacion'),
)

# En el frame de la base (x = 0.5 del mundo).
TAREA = (
    ('Traspaso 1-2',          (-0.250, 0.000, 0.060), 'arriba'),
    ('Traspaso 2-3',          (0.220, 0.000, 0.060), 'abajo'),
    ('Puesto de inspeccion',  (0.000, 0.200, 0.060), 'auto'),
)

TOLERANCIA = 1e-9


def _rango(a, b, n):
    return (a + (b - a) * i / (n - 1) for i in range(n))


def barrido(n=41):
    """Ida y vuelta sobre una rejilla de n x n x 5 posiciones articulares."""
    peor_p = peor_q = 0.0
    ensayadas = resueltas = 0
    for t1 in _rango(cin.T1_MIN, cin.T1_MAX, n):
        for t2 in _rango(-cin.T2_MAX, cin.T2_MAX, n):
            if abs(t2) < 1e-6:
                continue           # brazo estirado: las dos ramas coinciden
            for d3 in _rango(0.0, cin.D3_MAX, 5):
                q = (t1, t2, d3)
                _, p = cin.directa(q)
                ensayadas += 1
                rama = 'arriba' if t2 > 0 else 'abajo'
                q2, ok, _ = cin.inversa(p, rama=rama)
                if not ok:
                    continue
                resueltas += 1
                peor_p = max(peor_p, cin.error_cierre(p, q2))
                dq = max(abs(math.remainder(a - b, 2*math.pi)) if i < 2
                         else abs(a - b) for i, (a, b) in enumerate(zip(q, q2)))
                peor_q = max(peor_q, dq)
    return {'ensayadas': ensayadas, 'resueltas': resueltas,
            'peor': peor_p, 'peor_q': peor_q}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--rejilla', type=int, default=61)
    args = ap.parse_args(argv)

    r = barrido(args.rejilla)
    print('1. Ida y vuelta: %d de %d resueltas, peor cierre %.2e m, peor '
          'diferencia articular %.2e'
          % (r['resueltas'], r['ensayadas'], r['peor'], r['peor_q']))

    print('2. Puntos imposibles:')
    for nombre, punto, esperado in IMPOSIBLES:
        _, ok, motivo = cin.inversa(punto)
        marca = 'OK' if (not ok and motivo == esperado) else 'FALLO'
        print('   %-26s %-38s %s' % (nombre, motivo, marca))

    print('3. Puntos de tarea (frame de la base):')
    for nombre, punto, rama in TAREA:
        q, ok, motivo = cin.inversa(punto, rama=rama)
        if not ok:
            print('   %-22s INALCANZABLE: %s' % (nombre, motivo))
            continue
        print('   %-22s theta1 %7.2f  theta2 %7.2f  d3 %5.1f mm  cierre %.1e m'
              % (nombre, math.degrees(q[0]), math.degrees(q[1]), q[2]*1000,
                 cin.error_cierre(punto, q)))
    q, ok, _ = cin.inversa_camara((0.0, 0.200))
    print('   %-22s theta1 %7.2f  theta2 %7.2f  lente en (%.4f, %.4f)'
          % ('Camara sobre el puesto', math.degrees(q[0]),
             math.degrees(q[1]), *cin.camara(q)[:2]))
    print('Alcance: radio %.1f a %.1f mm, tool0 de z %.0f a %.0f mm'
          % (cin.R_MIN*1000, cin.R_MAX*1000, (cin.Z0 - cin.D3_MAX)*1000,
             cin.Z0*1000))


if __name__ == '__main__':
    main()
