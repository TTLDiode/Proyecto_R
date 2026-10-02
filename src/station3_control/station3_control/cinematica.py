"""
Cinematica de la Estacion 3 (RRP).

Migrado de Simulacion_dh.m, misma convencion DH y misma tabla de
parametros que el URDF (station3.urdf.xacro). Verificado numericamente
contra mediciones reales en Gazebo (tf2_echo world Prismatico_1):

  q = (-1.2706352, -1.4101057, -0.0116250) -> (x=-0.060, y=-0.140, z=0.050)

Convencion de la tabla DH (fila = [a, alpha, s, theta]):
  Frame 1 (base -> joint1):  a=0.0, alpha=0, s=0.0535,    theta=theta1 (variable)
  Frame 2 (joint1 -> joint2): a=0.1, alpha=0, s=0.026,     theta=theta2 (variable)
  Frame 3 (joint2 -> joint3): a=0.1, alpha=0, s=s3 (variable), theta=0
  Frame 4 (offset fijo):      a=0.0, alpha=0, s=-0.017875, theta=0

Limites articulares (iguales a station3.urdf.xacro):
  theta1: -90   a  +90   grados   (-1.570796 a 1.570796 rad)
  theta2: -127  a  +127  grados   (-2.216568 a 2.216568 rad)
  s3:     -0.0616 a 0.0 m
"""

import numpy as np

LIMITES_THETA1 = (-1.570796, 1.570796)
LIMITES_THETA2 = (-2.216568, 2.216568)
LIMITES_S3 = (-0.0616, 0.0)


def t_dh(a, alpha, s, theta):
    """Matriz homogenea DH de una fila de la tabla (misma forma que T_DH en MATLAB)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,     -st,     0,     a],
        [ca * st, ca * ct, -sa, -s * sa],
        [sa * st, sa * ct,  ca,  s * ca],
        [0,       0,        0,   1],
    ])


def cinematica_directa(theta1, theta2, s3):
    """
    Cinematica directa.

    Args:
        theta1, theta2: angulos de las articulaciones revolutas, en radianes.
        s3: desplazamiento de la articulacion prismatica, en metros.

    Returns:
        T: matriz homogenea 4x4 (numpy array) del efector final respecto a la base.
    """
    tabla_dh = [
        (0.0, 0.0, 0.0535,    theta1),
        (0.1, 0.0, 0.026,     theta2),
        (0.1, 0.0, s3,        0.0),
        (0.0, 0.0, -0.017875, 0.0),
    ]
    t = np.eye(4)
    for a, alpha, s, theta in tabla_dh:
        t = t @ t_dh(a, alpha, s, theta)
    return t


def posicion_efector(theta1, theta2, s3):
    """Devuelve solo (x, y, z) del efector final."""
    t = cinematica_directa(theta1, theta2, s3)
    return t[0, 3], t[1, 3], t[2, 3]


def cinematica_inversa(px, py, pz):
    """
    Cinematica inversa, forma cerrada (migrada de Simulacion_Cinematica_Inversa.m).

    Devuelve las dos ramas posibles del codo (acos(D) y -acos(D)).

    Args:
        px, py, pz: posicion deseada del efector final, en metros.

    Returns:
        soluciones: lista de 2 tuplas (theta1, theta2, s3) en rad/rad/m,
                     o None en la posicion de una rama si el punto esta
                     fuera del volumen de trabajo (|D| > 1).
        validas: lista de 2 booleanos, True si esa rama ademas respeta
                 los limites articulares de station3.urdf.xacro.
    """
    a12, a23 = 0.1, 0.1
    s1, s2, s4 = 0.0535, 0.026, -0.017875

    soluciones = [None, None]
    validas = [False, False]

    d = (px**2 + py**2 - a12**2 - a23**2) / (2 * a12 * a23)
    if abs(d) > 1:
        return soluciones, validas

    s3 = pz - s1 - s2 - s4
    s3_ok = LIMITES_S3[0] <= s3 <= LIMITES_S3[1]

    for k, theta2 in enumerate([np.arccos(d), -np.arccos(d)]):
        cos_theta1 = (px * (a12 + a23 * np.cos(theta2)) + py * (a23 * np.sin(theta2))) / (px**2 + py**2)
        sin_theta1 = (py * (a12 + a23 * np.cos(theta2)) - px * (a23 * np.sin(theta2))) / (px**2 + py**2)
        theta1 = np.arctan2(sin_theta1, cos_theta1)

        soluciones[k] = (theta1, theta2, s3)

        theta1_ok = LIMITES_THETA1[0] <= theta1 <= LIMITES_THETA1[1]
        theta2_ok = LIMITES_THETA2[0] <= theta2 <= LIMITES_THETA2[1]
        validas[k] = theta1_ok and theta2_ok and s3_ok

    return soluciones, validas
