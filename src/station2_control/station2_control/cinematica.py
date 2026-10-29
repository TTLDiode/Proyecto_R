"""Cinematica de la Estacion 2 (TRIADA), implementacion propia.

Misma estructura y mismos nombres que station1_control/cinematica.py, de la
que esta adaptada, y misma traduccion linea por linea que
matlab/station2/e2_*.m. No usa ninguna biblioteca de robotica: las matrices
homogeneas se construyen aqui y la inversa es cerrada.

Tabla de Denavit-Hartenberg, convencion estandar
A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)

    i   theta      d (m)     a (m)    alpha
    1   theta1 *   0.114     0.160    0
    2   theta2 *   0.006     0.140    pi
    3   0          d3 *      0        0

d1 = 0.114 es la altura del eje de theta1 sobre el suelo. d2 = 0.006 lleva
desde ese plano hasta tool0 con d3 = 0: el eslabon 2 esta 42 mm por encima y
el carro cuelga 36 mm por debajo de el (0.114 + 0.042 - 0.036 = 0.120).
alpha2 = pi voltea el eje z del sistema 2 hacia abajo, de modo que d3
positivo desciende.

Las constantes son las de station2_description/urdf/parametros.xacro, que a
su vez salen del CAD (docs/estacion2/urdf-datos-cad.md). comparar_moveit.py
compara esta cinematica contra el arbol TF que publica el simulador, de modo
que una divergencia entre las dos copias aparece como error medido.

Todas las funciones trabajan en el frame de la base de la estacion. Los
puntos de la linea vienen en el frame world y se pasan con a_base().
"""
import math

# ----------------------------------------------------------------------------
# Constantes de la maquina
# ----------------------------------------------------------------------------
A1 = 0.160          # eslabon 1, distancia entre ejes
A2 = 0.140          # eslabon 2, codo a vastago
D1 = 0.114          # altura del eje de theta1 sobre el suelo
D2 = 0.006          # del plano de theta1 a tool0 con d3 = 0

T1_MIN = math.radians(-30.0)
T1_MAX = math.radians(210.0)
T2_MAX = math.radians(135.0)
D3_MAX = 0.080

Z0 = D1 + D2                                        # 0.120 m
R_MAX = A1 + A2                                     # 0.300 m
R_MIN = math.sqrt(A1**2 + A2**2 + 2*A1*A2*math.cos(T2_MAX))   # 0.116 m

# Base de la estacion en el mundo, de line_bringup/config/line_poses.yaml
BASE = (0.500, 0.000, 0.000)

# Camara bajo el eslabon 2, en el frame del eslabon 2 (x hacia el vastago).
CAM_X = 0.070
CAM_Y = 0.009
CAM_Z = 0.151        # altura de la lente sobre el suelo, fija
CAM_L = math.hypot(CAM_X, CAM_Y)
CAM_FI = math.atan2(CAM_Y, CAM_X)

ARTICULACIONES = ('station2/joint_1', 'station2/joint_2', 'station2/joint_3')


def a_base(p):
    """Punto del frame world al frame de la base (la base no esta girada)."""
    return (p[0] - BASE[0], p[1] - BASE[1], p[2] - BASE[2])


def a_mundo(p):
    return (p[0] + BASE[0], p[1] + BASE[1], p[2] + BASE[2])


# ----------------------------------------------------------------------------
# Directa
# ----------------------------------------------------------------------------
def dh(theta, d, a, alpha):
    """Matriz homogenea de un eslabon, convencion estandar."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return ((ct, -st*ca,  st*sa, a*ct),
            (st,  ct*ca, -ct*sa, a*st),
            (0.0,    sa,     ca,    d),
            (0.0,   0.0,    0.0,  1.0))


def producto(A, B):
    return tuple(tuple(sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4))
                 for i in range(4))


def directa(q):
    """Cinematica directa en el frame de la base. Devuelve (T, (x, y, z)).

    Forma cerrada equivalente:
        x = a1 cos(t1) + a2 cos(t1+t2)
        y = a1 sen(t1) + a2 sen(t1+t2)
        z = d1 + d2 - d3
    """
    t1, t2, d3 = q
    T = producto(producto(dh(t1, D1, A1, 0.0),
                          dh(t2, D2, A2, math.pi)),
                 dh(0.0, d3, 0.0, 0.0))
    return T, (T[0][3], T[1][3], T[2][3])


def guinada(q):
    """Orientacion de la herramienta: giro puro en z, no libre (3 GDL)."""
    return q[0] + q[1]


def camara(q):
    """Posicion de la lente de la camara en el frame de la base."""
    t1, t2, _ = q
    xc = A1*math.cos(t1) + CAM_L*math.cos(t1 + t2 + CAM_FI)
    yc = A1*math.sin(t1) + CAM_L*math.sin(t1 + t2 + CAM_FI)
    return (xc, yc, CAM_Z)


# ----------------------------------------------------------------------------
# Inversa, en forma cerrada
# ----------------------------------------------------------------------------
def _dos_barras(x, y, l1, l2, desfase, orden):
    """Problema plano de dos barras con un desfase angular en la segunda.

    Devuelve la primera solucion (t1, t2) que cumpla los limites, probando las
    ramas en el orden pedido, o (None, motivo).
    """
    c2 = (x*x + y*y - l1*l1 - l2*l2) / (2.0*l1*l2)
    if abs(c2) > 1.0 + 1e-12:
        return None, 'radio fuera del alcance'
    c2 = min(max(c2, -1.0), 1.0)
    for codo in orden:
        psi = codo * math.acos(c2)               # angulo de la segunda barra
        t2 = psi - desfase
        if abs(t2) > T2_MAX + 1e-12:
            continue
        t1 = math.atan2(y, x) - math.atan2(l2*math.sin(psi),
                                           l1 + l2*math.cos(psi))
        t1 = (t1 + math.pi) % (2.0*math.pi) - math.pi
        # theta1 llega a 210 grados: hay que probar tambien la vuelta.
        for cand in (t1, t1 + 2.0*math.pi):
            if T1_MIN - 1e-12 <= cand <= T1_MAX + 1e-12:
                return (cand, t2), 'ok'
    return None, 'fuera de los limites de articulacion'


def inversa(pos, rama='auto'):
    """Cinematica inversa de tool0, en el frame de la base.

    Devuelve (q, ok, motivo). La altura se despeja sola, d3 = d1 + d2 - z, y
    el plano es el problema de dos barras con sus dos ramas de codo.

    Con 'auto' se prueba primero codo arriba (theta2 > 0) y despues codo
    abajo. Los dos puntos de traspaso caen cada uno en una rama distinta:
    el 1 -> 2, en (-0.250, 0) del frame de la base, solo admite codo arriba
    (codo abajo pediria theta1 = 211.1, mas alla de 210); el 2 -> 3, en
    (0.220, 0), solo admite codo abajo (codo arriba pediria theta1 = -39.4).
    """
    x, y, z = pos
    d3 = Z0 - z
    if d3 < -1e-9 or d3 > D3_MAX + 1e-9:
        return None, False, 'altura fuera de la carrera'
    d3 = min(max(d3, 0.0), D3_MAX)

    orden = {'auto': (+1, -1), 'arriba': (+1,), 'abajo': (-1,)}[rama]
    sol, motivo = _dos_barras(x, y, A1, A2, 0.0, orden)
    if sol is None:
        return None, False, motivo
    return (sol[0], sol[1], d3), True, 'ok'


def inversa_camara(xy, rama='auto'):
    """Pose articular que pone la lente de la camara sobre el punto (x, y).

    Es el mismo problema de dos barras, con la segunda barra medida hasta la
    lente (70.6 mm) y girada 7.3 grados respecto del eslabon 2. d3 se deja
    arriba, en 0, para que la mordaza no entre en la imagen.
    """
    orden = {'auto': (+1, -1), 'arriba': (+1,), 'abajo': (-1,)}[rama]
    sol, motivo = _dos_barras(xy[0], xy[1], A1, CAM_L, CAM_FI, orden)
    if sol is None:
        return None, False, motivo
    return (sol[0], sol[1], 0.0), True, 'ok'


def error_cierre(pos, q):
    """Distancia entre el punto pedido y el que da la directa de la solucion."""
    _, p = directa(q)
    return math.dist(p, pos)
