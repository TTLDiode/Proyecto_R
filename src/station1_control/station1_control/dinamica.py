"""Modelo dinamico del brazo de la Estacion 1, para dimensionar el tiempo de
los movimientos.

Por que hace falta. Un limite de aceleracion por articulacion supone que las
articulaciones son independientes, y en un brazo planar de dos eslabones no lo
son: el par que ve theta1 depende tambien de lo que haga theta2. Con el codo
quieto el modelo desacoplado basta, y es lo que se midio al ajustar los
limites; en cuanto los dos ejes se mueven a la vez deja de bastar.

El caso que lo dejo en evidencia es el paso de la bandeja al punto de traspaso.
La bandeja se alcanza con el codo arriba y el traspaso exige codo abajo, de modo
que theta2 tiene que barrer 3.12 rad y pasar por la extension completa. Con el
perfil dimensionado solo por aceleracion, theta1 se quedaba 0.10 rad por detras
de su consigna y el controlador abortaba la trayectoria. La cuenta explica por
que: el termino de Coriolis que theta2 induce sobre theta1 llega a 0.30 N.m
frente a los 0.389 N.m que deja pasar el driver.

Modelo. Los tres eslabones se agrupan en dos cuerpos, porque el carro y la
mordaza no giran respecto del eslabon 2 y su prismatica es vertical: su
desplazamiento no cambia ninguna inercia respecto de los ejes verticales.

    M11(t2) = I1 + I2 + m2 (a1^2 + r2^2 + 2 a1 r2 cos t2)
    M12(t2) = I2 + m2 (r2^2 + a1 r2 cos t2)
    M22     = I2 + m2 r2^2
    h(t2)   = m2 a1 r2 sin t2

    tau1 = M11 dd1 + M12 dd2 - h (2 d1 d2 + d2^2)
    tau2 = M12 dd1 + M22 dd2 + h d1^2

La gravedad no aparece en ninguna de las dos: los dos ejes de giro son
verticales y el peso no produce par respecto de ellos. Es la ventaja de la
configuracion SCARA y conviene decirlo en la sustentacion, porque es la razon
por la que esta estacion no necesita frenos ni compensacion de gravedad en las
rotacionales.

Los parametros salen de sumar las piezas del CAD recogidas en parametros.xacro,
no de medidas nuevas.
"""
import math

# --- Cuerpo 1: eslabon 1 ---
M1 = 0.4954
R1 = 0.05405                      # centro de masa sobre el eje de theta1
I1_CM = 2.135e-3
I1 = I1_CM + M1 * R1**2           # respecto del eje de theta1, 3.582e-3

# --- Cuerpo 2: eslabon 2 + carro + mordaza, todo lo que cuelga del codo ---
M2 = 0.8560 + 0.1940 + 0.0272     # 1.0772 kg
R2 = 0.09859                      # centro de masa del conjunto sobre el codo
I2 = 2.711e-3                     # respecto de ese centro de masa

A1 = 0.180                        # distancia entre ejes del eslabon 1

# Par disponible en cada rotacional. No es el del motor, es el que deja pasar
# el TB6612FNG a 1.2 A: ver parametros.xacro.
TAU_MAX = 0.389

# Fraccion del par que se reserva para el lazo de posicion y para vencer
# amortiguamiento y friccion. Sale de la misma medida que fijo la aceleracion
# util en el 55 % de la teorica.
MARGEN_PAR = 0.55


def inercia(t2):
    """Matriz de inercia del brazo en funcion del angulo del codo."""
    c = math.cos(t2)
    m11 = I1 + I2 + M2*(A1*A1 + R2*R2 + 2*A1*R2*c)
    m12 = I2 + M2*(R2*R2 + A1*R2*c)
    m22 = I2 + M2*R2*R2
    return m11, m12, m22


def acoplamiento(t2):
    """Coeficiente de los terminos centrifugo y de Coriolis."""
    return M2 * A1 * R2 * math.sin(t2)


def pares(q, qd, qdd):
    """Pares en las dos rotacionales para un estado dado."""
    m11, m12, m22 = inercia(q[1])
    h = acoplamiento(q[1])
    tau1 = m11*qdd[0] + m12*qdd[1] - h*(2*qd[0]*qd[1] + qd[1]*qd[1])
    tau2 = m12*qdd[0] + m22*qdd[1] + h*qd[0]*qd[0]
    return tau1, tau2


def par_maximo(q0, q1, T, muestras=60):
    """Par maximo que exige recorrer q0 -> q1 en T con el perfil quintico."""
    peor = 0.0
    for k in range(muestras + 1):
        tau_n = k / muestras
        s = 10*tau_n**3 - 15*tau_n**4 + 6*tau_n**5
        ds = (30*tau_n**2 - 60*tau_n**3 + 30*tau_n**4) / T
        dds = (60*tau_n - 180*tau_n**2 + 120*tau_n**3) / (T*T)
        q = [a + (b-a)*s for a, b in zip(q0, q1)]
        qd = [(b-a)*ds for a, b in zip(q0, q1)]
        qdd = [(b-a)*dds for a, b in zip(q0, q1)]
        t1, t2 = pares(q, qd, qdd)
        peor = max(peor, abs(t1), abs(t2))
    return peor


def tiempo_minimo(q0, q1, T_inicial):
    """Estira T hasta que el par cabe en el presupuesto.

    Con el perfil quintico las velocidades van como 1/T y las aceleraciones
    como 1/T^2, y los dos terminos del par, el inercial y el de Coriolis, van
    como 1/T^2. El par maximo es por tanto C/T^2 exacto, de modo que el factor
    de estirado sale de una sola evaluacion y no hace falta iterar.
    """
    presupuesto = TAU_MAX * MARGEN_PAR
    tau = par_maximo(q0, q1, T_inicial)
    if tau <= presupuesto:
        return T_inicial, tau
    T = T_inicial * math.sqrt(tau / presupuesto)
    return T, par_maximo(q0, q1, T)
