"""Cinematica de la Estacion 1, implementacion propia.

Esta es la traduccion a Python de matlab/station1/*.m, linea por linea y con
los mismos nombres, para que el codigo que corre en la simulacion y el que se
verifica en MATLAB sean el mismo algoritmo y no dos parecidos. No usa ninguna
funcion de biblioteca de robotica: las matrices homogeneas se construyen aqui
y la inversa es cerrada.

Tabla de Denavit-Hartenberg, convencion estandar
A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)

    i   theta      d (m)     a (m)    alpha
    1   theta1 *   0.173     0.180    0
    2   theta2 *  -0.053     0.140    pi
    3   0          d3 *      0        0

alpha2 = pi voltea el eje z del sistema 2 hacia abajo, de modo que d3 positivo
desciende sin necesidad de meter un signo a mano.

Las constantes son las mismas de station1_description/urdf/parametros.xacro.
Estan escritas aqui y no leidas del URDF a proposito: el enunciado pide una
implementacion propia y verificable, y una implementacion que dependiese del
analizador de xacro no se podria ejecutar sin ROS. La coherencia entre las dos
copias no se confia a la disciplina: comparar_moveit.py compara esta
cinematica contra el arbol TF que publica el simulador a partir del URDF, de
modo que cualquier divergencia aparece como error en la tabla de resultados.
"""
import math

# ----------------------------------------------------------------------------
# Constantes de la maquina
# ----------------------------------------------------------------------------
A1 = 0.180          # eslabon 1, distancia entre ejes
A2 = 0.140          # eslabon 2, distancia entre ejes
D1 = 0.173          # altura del eje de theta1 sobre la base
D2 = -0.053         # del plano del brazo a la herramienta con d3 = 0

T1_MIN = math.radians(-15.0)
T1_MAX = math.radians(185.0)
T2_MAX = math.radians(135.0)
D3_MAX = 0.084

Z0 = D1 + D2                                        # 0.120 m
R_MAX = A1 + A2                                     # 0.320 m
R_MIN = math.sqrt(A1**2 + A2**2 + 2*A1*A2*math.cos(T2_MAX))   # 0.128 m

ARTICULACIONES = ('station1/joint_1', 'station1/joint_2', 'station1/joint_3')


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
    """Cinematica directa. Devuelve (T, (x, y, z)).

    La forma cerrada equivalente es
        x = a1 cos(t1) + a2 cos(t1+t2)
        y = a1 sen(t1) + a2 sen(t1+t2)
        z = d1 + d2 - d3
    es decir, el plano depende solo de las rotacionales y la altura solo de la
    prismatica. Los dos problemas estan desacoplados, y de ahi que la inversa
    salga cerrada.
    """
    t1, t2, d3 = q
    T = producto(producto(dh(t1, D1, A1, 0.0),
                          dh(t2, D2, A2, math.pi)),
                 dh(0.0, d3, 0.0, 0.0))
    return T, (T[0][3], T[1][3], T[2][3])


def guinada(q):
    """Orientacion alcanzable de la herramienta: es un giro puro en z.

    No es un grado de libertad libre. Con tres articulaciones y la posicion
    fijada, la guinada queda determinada por la rama del codo. Es la razon por
    la que el solucionador de MoveIt va configurado en modo position_only_ik.
    """
    return q[0] + q[1]


# ----------------------------------------------------------------------------
# Inversa, en forma cerrada
# ----------------------------------------------------------------------------
def inversa(pos, rama='auto'):
    """Cinematica inversa. Devuelve (q, ok, motivo).

    La altura se despeja sola, porque z no depende de las rotacionales:
        d3 = d1 + d2 - z

    El plano es el problema de dos barras, con dos soluciones simetricas
    respecto de la recta que une el eje con la herramienta:
        cos(t2) = (r^2 - a1^2 - a2^2) / (2 a1 a2)
        t2      = +/- acos(...)                     codo arriba o abajo
        t1      = atan2(y,x) - atan2(a2 sen t2, a1 + a2 cos t2)

    Eleccion de rama. Con 'auto' se prueba primero codo arriba y solo se pasa a
    codo abajo si la primera no cumple los limites. El criterio no es de
    frecuencia sino de despeje: la rama de codo arriba aleja el eslabon 2 del
    pilar del final de carrera, que esta 88 mm por detras del eje de theta1.
    De los puntos alcanzables del plano, el 18.2 % solo admite codo arriba, el
    18.2 % solo codo abajo y el 63.6 % admite los dos. El punto de traspaso a
    la Estacion 2 pertenece al primer grupo por el lado contrario: exige codo
    abajo, porque la rama de codo arriba pediria theta1 = -33.2 grados y el
    limite inferior esta en -15.
    """
    x, y, z = pos
    d3 = Z0 - z
    if d3 < -1e-9 or d3 > D3_MAX + 1e-9:
        return None, False, 'altura fuera de la carrera'
    d3 = min(max(d3, 0.0), D3_MAX)      # recorta el redondeo en los topes

    c2 = (x*x + y*y - A1*A1 - A2*A2) / (2.0*A1*A2)
    if abs(c2) > 1.0 + 1e-12:
        return None, False, 'radio fuera del alcance'
    c2 = min(max(c2, -1.0), 1.0)        # idem en el borde del anillo

    orden = {'auto': (+1, -1), 'arriba': (+1,), 'abajo': (-1,)}[rama]
    for codo in orden:
        t2 = codo * math.acos(c2)
        if abs(t2) > T2_MAX + 1e-12:
            continue
        t1 = math.atan2(y, x) - math.atan2(A2*math.sin(t2), A1 + A2*math.cos(t2))
        t1 = (t1 + math.pi) % (2.0*math.pi) - math.pi
        # theta1 llega a 185 grados, de modo que hay que probar tambien la
        # vuelta completa antes de descartar el punto.
        for cand in (t1, t1 + 2.0*math.pi):
            if T1_MIN - 1e-12 <= cand <= T1_MAX + 1e-12:
                return (cand, t2, d3), True, 'ok'

    return None, False, 'fuera de los limites de articulacion'


def error_cierre(pos, q):
    """Distancia entre el punto pedido y el que da la directa de la solucion."""
    _, p = directa(q)
    return math.dist(p, pos)
