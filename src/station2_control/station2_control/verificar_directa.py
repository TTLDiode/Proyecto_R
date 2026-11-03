"""Directa propia contra los codigos de clase de MATLAB, pose por pose.

    cd ~/Proyecto_R/src/station2_control
    python3 -m station2_control.verificar_directa

Contraparte en Python de las dos corridas de MATLAB de la Entrega 2:

  1. Reposo (brazo estirado, theta1 = 90, theta2 = 0, d3 = 0), la misma pose
     de ri_dk_ms3r_TH_DH.m con la tabla DH de la estacion: imprime la T
     0-herramienta, que debe ser igual a la T(:,:,4) de MATLAB.
  2. Las cinco configuraciones de e2_ri_dk_TH_DH.m (las que devolvio MoveIt
     en comparar_moveit): posicion de la directa, punto pedido y error, en el
     marco de la base y en mm, con el mismo formato que MATLAB.

Solo usa cinematica.directa, sin ROS: no hace falta compilar ni lanzar nada.
"""
import math

from station2_control.cinematica import directa

NOMBRES = ('traspaso_12', 'inspeccion', 'traspaso_23', 'libre_1', 'libre_2')
Q_MOVEIT = ((148.90, 67.29, 60), (45.95, 96.67, 60), (39.40, -85.90, 60),
            (136.15, -52.46, 40), (67.97, -67.28, 20))    # grados, grados, mm
P_OBJ = ((-250, 0, 60), (0, 200, 60), (220, 0, 60),
         (-100, 250, 80), (200, 150, 100))                # mm, marco de la base


def t_en_mm(q_grados_mm):
    """T 0-herramienta con la posicion en mm, como la da MATLAB."""
    t1, t2, d3 = q_grados_mm
    T, _ = directa((math.radians(t1), math.radians(t2), d3 / 1000.0))
    return [[v * 1000.0 if (j == 3 and i < 3) else v for j, v in enumerate(fila)]
            for i, fila in enumerate(T)]


def imprimir_t(T):
    for fila in T:
        print('  ' + ' '.join('%10.4f' % (v + 0.0) for v in fila))


def main():
    print('1) Reposo, brazo estirado: theta1 = 90, theta2 = 0, d3 = 0')
    print('T 0-herramienta (mm):')
    imprimir_t(t_en_mm((90.0, 0.0, 0.0)))

    print('\n2) Configuraciones de e2_ri_dk_TH_DH.m')
    print('%-12s %28s %28s %10s' % ('punto', 'p directa (mm)', 'p objetivo (mm)',
                                    'error (mm)'))
    for nombre, q, obj in zip(NOMBRES, Q_MOVEIT, P_OBJ):
        p = [fila[3] for fila in t_en_mm(q)[:3]]
        print('%-12s (%8.3f, %8.3f, %8.3f) (%8.3f, %8.3f, %8.3f) %10.4f'
              % (nombre, *p, *obj, math.dist(p, obj)))
    print('T 0-herramienta en traspaso_12:')
    imprimir_t(t_en_mm(Q_MOVEIT[0]))


if __name__ == '__main__':
    main()
