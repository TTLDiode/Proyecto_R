"""Generador de trayectorias punto a punto, implementacion propia.

Es la mitad de control del codigo propio: la cinematica inversa dice a que
posicion articular hay que ir y esto dice como llegar. Sin ello el controlador
recibiria un salto y el simulador lo seguiria a la maxima aceleracion que le
permitiese el motor, que es justo lo que los limites tratan de evitar.

Perfil quintico sincronizado. Todas las articulaciones recorren su trayecto
con el mismo polinomio normalizado

    s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5,     tau = t / T

y llegan a la vez. El quintico se elige porque anula velocidad y aceleracion
en los dos extremos: el movimiento arranca y termina sin escalon de
aceleracion, y por tanto sin el pico de jerk que un perfil trapezoidal o
cubico deja en los bordes. En una maquina con correas dentadas ese escalon es
precisamente lo que las hace saltar dientes.

Sus derivadas normalizadas tienen maximos conocidos:

    |s'|max  = 1.875 / T        ->  T >= 1.875 * dq / v_max
    |s''|max = 5.7735 / T^2     ->  T >= sqrt(5.7735 * dq / a_max)

de modo que el tiempo minimo que respeta los dos limites sale de una formula
cerrada, sin busqueda iterativa: se toma el mayor de los dos por articulacion
y el mayor de todos entre articulaciones.

Los limites por defecto son los de
station1_moveit_config/config/joint_limits.yaml, que a su vez salen del par
que deja pasar el TB6612FNG dividido por la inercia del CAD.
"""
import math

from station1_control import dinamica as din

# (v_max, a_max) por articulacion, en el orden theta1, theta2, d3.
#
# Las velocidades son las de la hoja de datos. Las aceleraciones NO son las que
# sale de dividir el par por la inercia: son las que el eje sigue de verdad,
# medidas contra el simulador. La diferencia importa y conviene saberla
# explicar.
#
# El par que deja pasar el TB6612FNG, 0.389 N.m, dividido por la inercia del
# brazo extendido, 0.0893 kg.m2, da 4.35 rad/s2. Ese numero supone que todo el
# par va a acelerar y no queda nada para nada mas, lo cual no es cierto: el
# actuador tiene ademas que vencer el amortiguamiento y la friccion de la
# articulacion y, sobre todo, conservar autoridad para corregir su propio error
# de seguimiento. Medido sobre un giro de 1.324 rad de theta1:
#
#     a pedida (rad/s2)   4.00     3.00     2.40     2.00     1.50
#     error max (mrad)     542      424     10.4     10.7      5.9
#
# Es un codo, no una degradacion suave: por encima de 2.4 rad/s2 el actuador
# satura y el eje deja de seguir la consigna por completo. Se toman 2.4, el
# 55 % del limite teorico, y ese 55 % es el margen de par del lazo.
LIMITES = ((20.944, 2.4),
           (20.944, 15.0),
           (0.0267, 0.3))

# Reserva de velocidad. El perfil no se planifica contra el limite fisico sino
# contra el 70 % de el, y el 30 % restante se deja para el lazo de posicion.
#
# No es prudencia generica: es el resultado de una medida. El actuador simulado
# sigue la consigna con un lazo proporcional que responde con velocidad, de modo
# que para corregir un retraso tiene que pedir mas velocidad que la de la
# referencia. Cuando el perfil se acerca al limite esa reserva se agota y el
# error deja de ser proporcional a la velocidad para dispararse.
#
# Medido sobre la prismatica bajando 40 mm, que es la unica articulacion cuyo
# limite de velocidad es realmente alcanzable:
#
#     v pico (m/s)   0.0050   0.0100   0.0160   0.0214   0.0267
#     error max (mm)   0.04     0.08     0.34     2.21     6.78
#
# El limite de la articulacion son 0.0267 m/s. Al 100 % el error llega a 6.8 mm
# y el controlador aborta la trayectoria por violacion de tolerancia; al 70 %,
# 0.0187 m/s, se queda por debajo de 1 mm. Subiendo el error es diez veces
# menor a igualdad de velocidad, porque entonces la gravedad frena en lugar de
# empujar, pero se aplica la misma reserva en los dos sentidos.
#
# Las rotacionales no se ven afectadas: su limite es 20.944 rad/s y el
# movimiento mas rapido de la estacion no pasa de 2 rad/s, de modo que les
# sobra reserva. Se les aplica el mismo factor por coherencia, sin efecto
# practico, porque su tiempo lo fija la aceleracion y no la velocidad.
MARGEN_VELOCIDAD = 0.7

_PICO_VEL = 1.875
_PICO_ACE = 5.7735


def duracion(q0, q1, limites=LIMITES, escala_vel=1.0, escala_ace=1.0):
    """Tiempo minimo del movimiento que cabe en los limites de la maquina.

    Se resuelve en dos pasos. Primero los limites por articulacion, que son
    independientes entre si y dan una cota cerrada. Despues el presupuesto de
    par del modelo dinamico, que es el que atrapa el acoplamiento entre theta1
    y theta2 y que ninguna cota por articulacion puede ver.
    """
    T = _duracion_desacoplada(q0, q1, limites, escala_vel, escala_ace)
    T, _ = din.tiempo_minimo(q0, q1, T)
    return T


def _duracion_desacoplada(q0, q1, limites=LIMITES, escala_vel=1.0,
                          escala_ace=1.0):
    """Cota por velocidad y aceleracion de cada articulacion por separado."""
    t = 0.0
    for a, b, (v_max, a_max) in zip(q0, q1, limites):
        dq = abs(b - a)
        if dq < 1e-12:
            continue
        t = max(t,
                _PICO_VEL * dq / (v_max * MARGEN_VELOCIDAD * escala_vel),
                math.sqrt(_PICO_ACE * dq / (a_max * escala_ace)))
    return max(t, 0.05)        # un movimiento nulo no dura cero


def perfil(tau):
    """Devuelve (s, s', s'') del quintico normalizado en tau = t/T."""
    s = 10*tau**3 - 15*tau**4 + 6*tau**5
    ds = 30*tau**2 - 60*tau**3 + 30*tau**4
    dds = 60*tau - 180*tau**2 + 120*tau**3
    return s, ds, dds


def muestrear(q0, q1, T, frecuencia=50.0):
    """Discretiza el movimiento en (t, q, qd) a la frecuencia pedida.

    Se entrega muestreado y no como dos extremos porque el controlador
    interpola con splines entre los puntos que recibe: darle solo el principio
    y el final le dejaria elegir a el el perfil, y entonces el perfil no seria
    propio.
    """
    # Arranca en k = 1 y no en k = 0: un punto en t = 0 coincide con el
    # instante en que el controlador recibe el mensaje y este lo descarta por
    # no ser estrictamente creciente en el tiempo.
    n = max(2, int(round(T * frecuencia)))
    puntos = []
    for k in range(1, n + 1):
        t = T * k / n
        s, ds, _ = perfil(t / T)
        q = [a + (b - a) * s for a, b in zip(q0, q1)]
        qd = [(b - a) * ds / T for a, b in zip(q0, q1)]
        puntos.append((t, q, qd))
    return puntos


def verificar(q0, q1, T):
    """Picos alcanzados, para comprobar contra los limites."""
    picos_v, picos_a = [], []
    for a, b in zip(q0, q1):
        dq = b - a
        picos_v.append(abs(dq) * _PICO_VEL / T)
        picos_a.append(abs(dq) * _PICO_ACE / T**2)
    return picos_v, picos_a
