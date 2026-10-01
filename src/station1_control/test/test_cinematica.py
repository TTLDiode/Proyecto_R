"""La prueba de ida y vuelta, en forma de aserciones.

Corre con `colcon test` y con `pytest`. El barrido se hace con un paso mas
grueso que el del informe para que la construccion no se alargue: el objetivo
aqui es detectar una regresion, no producir la cifra que se reporta.
"""
import math

from station1_control import cinematica as cin
from station1_control import prueba_cinematica as pr


def test_cierre_en_todo_el_espacio():
    r = pr.barrido(paso_r=0.006, paso_ang=math.radians(5), paso_z=0.012)
    assert r['resueltas'] == r['ensayadas'], 'hay poses alcanzables sin solucion'
    assert r['peor'] < pr.TOLERANCIA, 'la inversa no cierra sobre la directa'


def test_rechaza_lo_imposible():
    for nombre, punto, esperado in pr.IMPOSIBLES:
        q, ok, motivo = cin.inversa(punto)
        assert not ok, f'{nombre} deberia rechazarse'
        assert motivo == esperado, f'{nombre}: motivo inesperado, {motivo}'


def test_puntos_de_tarea_alcanzables():
    for nombre, punto in pr.TAREA:
        q, ok, motivo = cin.inversa(punto)
        assert ok, f'{nombre} deberia ser alcanzable, dice: {motivo}'
        assert cin.error_cierre(punto, q) < pr.TOLERANCIA


def test_limites_respetados():
    for nombre, punto in pr.TAREA:
        q, ok, _ = cin.inversa(punto)
        assert cin.T1_MIN - 1e-9 <= q[0] <= cin.T1_MAX + 1e-9
        assert abs(q[1]) <= cin.T2_MAX + 1e-9
        assert 0.0 - 1e-9 <= q[2] <= cin.D3_MAX + 1e-9


def test_el_traspaso_exige_codo_abajo():
    """Es el punto que rompe la regla general, y conviene que quede fijado."""
    punto = (0.250, 0.000, 0.060)
    q, ok, _ = cin.inversa(punto, rama='abajo')
    assert ok and q[1] < 0
    _, ok_arriba, motivo = cin.inversa(punto, rama='arriba')
    assert not ok_arriba, 'la rama de codo arriba deberia salirse de theta1_min'
