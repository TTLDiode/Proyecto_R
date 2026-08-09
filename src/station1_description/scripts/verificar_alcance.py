#!/usr/bin/env python3
"""Verifica que la Estacion 1 alcanza todos sus puntos de tarea.

Comprueba la cobertura del espacio de trabajo exigida por la guia. Lee la
geometria de la linea de config/line_poses.yaml y los parametros del
manipulador de urdf/parametros.xacro, de modo que si cualquiera de los dos
cambia, la verificacion se hace sobre los valores nuevos.

    python3 verificar_alcance.py

Devuelve codigo de salida distinto de cero si algun punto queda fuera, lo que
permite usarlo como comprobacion automatica.
"""
import math
import os
import re
import sys

import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
PARAMETROS = os.path.join(AQUI, '..', 'urdf', 'parametros.xacro')
POSES = os.path.join(AQUI, '..', '..', 'line_bringup', 'config', 'line_poses.yaml')


def leer_parametros(ruta):
    """Extrae las propiedades numericas del xacro sin procesarlo entero."""
    texto = open(ruta).read()
    patron = r'<xacro:property\s+name="(\w+)"\s+value="([-\d.]+)"\s*/>'
    return {n: float(v) for n, v in re.findall(patron, texto)}


def main():
    p = leer_parametros(PARAMETROS)
    poses = yaml.safe_load(open(POSES))

    l1, l2 = p['l1'], p['l2']
    lim_t1, lim_t2 = p['lim_theta1'], p['lim_theta2']
    z_brazo = p['altura_columna'] - p['brazo_espesor']
    z_tool = lambda d3: z_brazo - d3 - p['gripper_z']

    # El codo limitado impide plegar la cadena por completo, de modo que el
    # radio minimo util es mayor que la diferencia de longitudes.
    r_min = math.sqrt(l1**2 + l2**2 + 2*l1*l2*math.cos(lim_t2))
    r_max = l1 + l2

    base = poses['station_bases']['station1']
    bandeja = poses['input_tray']
    traspaso = poses['handoff_1_2']['position']

    puntos = []
    cx, cy, cz = bandeja['center']['x'], bandeja['center']['y'], bandeja['center']['z']
    hx, hy = bandeja['size']['x']/2, bandeja['size']['y']/2
    puntos.append(('Centro de la bandeja', cx, cy, cz))
    for sx in (-1, 1):
        for sy in (-1, 1):
            puntos.append((f'Esquina de la bandeja ({sx:+d},{sy:+d})',
                           cx + sx*hx, cy + sy*hy, cz))
    puntos.append(('Traspaso a la Estacion 2',
                   traspaso['x'], traspaso['y'], traspaso['z']))

    print('Estacion 1 - Cobertura del espacio de trabajo')
    print(f'  Configuracion RRP, l1 = {l1*1000:.0f} mm, l2 = {l2*1000:.0f} mm')
    print(f'  Anillo alcanzable: r de {r_min*1000:.0f} a {r_max*1000:.0f} mm')
    print(f'  Altura del efector: z de {z_tool(p["carrera_d3"])*1000:.0f} '
          f'a {z_tool(0)*1000:.0f} mm')
    print(f'  Altura maxima de la estacion: {(z_brazo + p["carrera_d3"])*1000:.0f} mm')
    print()
    print(f'  {"Punto de tarea":<30}{"r (mm)":>8}{"theta1":>9}{"theta2":>9}{"d3 (mm)":>9}  ')
    print('  ' + '-'*68)

    fallos = 0
    for nombre, x, y, z in puntos:
        xr, yr = x - base['x'], y - base['y']
        r = math.hypot(xr, yr)
        d3 = z_brazo - p['gripper_z'] - z
        sol = None
        if r_min <= r <= r_max and 0.0 <= d3 <= p['carrera_d3']:
            c2 = (r*r - l1*l1 - l2*l2) / (2*l1*l2)
            if abs(c2) <= 1.0:
                t2 = math.acos(c2)
                t1 = math.atan2(yr, xr) - math.atan2(l2*math.sin(t2), l1 + l2*math.cos(t2))
                if abs(t1) <= lim_t1 and abs(t2) <= lim_t2:
                    sol = (math.degrees(t1), math.degrees(t2), d3)
        if sol:
            print(f'  {nombre:<30}{r*1000:8.0f}{sol[0]:9.1f}{sol[1]:9.1f}{sol[2]*1000:9.1f}  si')
        else:
            print(f'  {nombre:<30}{r*1000:8.0f}{"-":>9}{"-":>9}{"-":>9}  NO')
            fallos += 1

    print()
    if fallos:
        print(f'  {fallos} punto(s) fuera del espacio de trabajo alcanzable.')
        return 1
    print('  Todos los puntos de tarea son alcanzables.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
