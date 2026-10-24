"""Ida y vuelta de la cinematica en forma de aserciones (pytest / colcon test).

Adaptado de station1_control/test/test_cinematica.py.
"""
import math

from station2_control import cinematica as cin
from station2_control import prueba_cinematica as pr


def test_cierre_en_todo_el_espacio():
    r = pr.barrido(n=25)
    assert r['resueltas'] == r['ensayadas'], 'hay poses alcanzables sin solucion'
    assert r['peor'] < pr.TOLERANCIA
    assert r['peor_q'] < 1e-9


def test_rechaza_lo_imposible():
    for nombre, punto, esperado in pr.IMPOSIBLES:
        _, ok, motivo = cin.inversa(punto)
        assert not ok, f'{nombre} deberia rechazarse'
        assert motivo == esperado, f'{nombre}: motivo inesperado, {motivo}'


def test_puntos_de_tarea():
    for nombre, punto, rama in pr.TAREA:
        q, ok, motivo = cin.inversa(punto, rama=rama)
        assert ok, f'{nombre}: {motivo}'
        assert cin.error_cierre(punto, q) < pr.TOLERANCIA
        assert cin.T1_MIN - 1e-9 <= q[0] <= cin.T1_MAX + 1e-9
        assert abs(q[1]) <= cin.T2_MAX + 1e-9


def test_cada_traspaso_tiene_una_sola_rama():
    """El 1-2 solo con codo arriba y el 2-3 solo con codo abajo."""
    assert not cin.inversa((-0.250, 0.0, 0.060), rama='abajo')[1]
    assert not cin.inversa((0.220, 0.0, 0.060), rama='arriba')[1]


def test_camara_no_llega_al_traspaso_12_pero_si_al_puesto():
    assert not cin.inversa_camara((-0.250, 0.0))[1]
    q, ok, _ = cin.inversa_camara((0.0, 0.200))
    assert ok
    xc, yc, _ = cin.camara(q)
    assert math.hypot(xc - 0.0, yc - 0.200) < 1e-9


def test_directa_coincide_con_la_forma_cerrada():
    q = (0.3, -1.1, 0.05)
    _, p = cin.directa(q)
    x = cin.A1*math.cos(q[0]) + cin.A2*math.cos(q[0] + q[1])
    y = cin.A1*math.sin(q[0]) + cin.A2*math.sin(q[0] + q[1])
    assert math.dist(p, (x, y, cin.Z0 - q[2])) < 1e-12
