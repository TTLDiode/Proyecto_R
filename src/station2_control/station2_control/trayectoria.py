"""Generador de trayectorias punto a punto, implementacion propia.

Adaptado de station1_control/trayectoria.py: el mismo perfil quintico
sincronizado, sin el modelo dinamico acoplado, que en la Estacion 2 no hace
falta porque las tres articulaciones van sobradas de par (ver LIMITES).

    s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5,     tau = t / T

Anula velocidad y aceleracion en los extremos, de modo que el movimiento
arranca y termina sin escalon de aceleracion: en una maquina de correas
dentadas ese escalon es lo que hace saltar dientes.

Maximos de las derivadas normalizadas:

    |s'|max  = 1.875 / T        ->  T >= 1.875 * dq / v_max
    |s''|max = 5.7735 / T^2     ->  T >= sqrt(5.7735 * dq / a_max)

El tiempo minimo que respeta los dos limites sale cerrado: el mayor de los
dos por articulacion y el mayor de todos entre articulaciones.
"""
import math

# (v_max, a_max) por articulacion, en el orden theta1, theta2, d3.
#
# Velocidades: Pololu 37D 50:1 a 12 V, 20.944 rad/s en el eje del motor, por
# la relacion de cada articulacion (parametros.xacro):
#   theta1  3:1                         6.981 rad/s
#   theta2  1:1                        20.944 rad/s
#   d3      1:3 y husillo de 8 mm       0.080 m/s
#
# Aceleraciones: el par que deja pasar el TB6612FNG (0.389 N.m en el motor, 1.2
# A) por la relacion, dividido por la inercia del CAD con el brazo extendido,
# y reducido con margen amplio para dejar par al lazo de posicion y no
# arrastrar la pieza acoplada en la simulacion:
#   theta1  1.167 N.m / 0.075 kg m2 = 15.6 rad/s2   -> 4.0
#   theta2  0.389 N.m / 0.020 kg m2 = 19.5 rad/s2   -> 4.0
#   d3      44 N de empuje contra 3 N de peso        -> 0.5 m/s2
LIMITES = ((6.981, 4.0),
           (20.944, 4.0),
           (0.080, 0.5))

# Reserva de velocidad para el lazo de posicion del actuador simulado, la
# misma que en la Estacion 1 (alli medida: al 100 % el error se dispara).
MARGEN_VELOCIDAD = 0.7

_PICO_VEL = 1.875
_PICO_ACE = 5.7735


def duracion(q0, q1, limites=LIMITES, escala_vel=1.0, escala_ace=1.0):
    """Tiempo minimo del movimiento que cabe en los limites de la maquina."""
    t = 0.0
    for a, b, (v_max, a_max) in zip(q0, q1, limites):
        dq = abs(b - a)
        if dq < 1e-12:
            continue
        t = max(t,
                _PICO_VEL * dq / (v_max * MARGEN_VELOCIDAD * escala_vel),
                math.sqrt(_PICO_ACE * dq / (a_max * escala_ace)))
    return max(t, 0.2)


def perfil(tau):
    """Devuelve (s, s', s'') del quintico normalizado en tau = t/T."""
    s = 10*tau**3 - 15*tau**4 + 6*tau**5
    ds = 30*tau**2 - 60*tau**3 + 30*tau**4
    dds = 60*tau - 180*tau**2 + 120*tau**3
    return s, ds, dds


def muestrear(q0, q1, T, frecuencia=50.0):
    """Discretiza el movimiento en (t, q, qd) a la frecuencia pedida.

    Arranca en k = 1: un punto en t = 0 lo descarta el controlador por no
    ser estrictamente creciente en el tiempo.
    """
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
