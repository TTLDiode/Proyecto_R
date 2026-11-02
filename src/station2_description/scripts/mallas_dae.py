"""Triangulos del STEP (step_a_mallas.py) -> una malla Collada por eslabon, con color.

Uso: python mallas_dae.py <mallas.pkl> <carpeta_salida>

- El STEP esta en la pose de reposo (theta1 = 90, theta2 = 0, d3 = 0, pinza a
  60 mm): todo lo que no es la base se gira -90 grados en z para llevarlo a la
  pose cero, y luego se pasa al marco de cada eslabon (los mismos origenes que
  cad/station2/urdf_cad/scripts/mallas_fusion.py), en metros.
- Colores: los del STEP, salvo el acrilico (claro y transparente, como el
  'Acrilico, claro' de Fusion, que el STEP no trae), la tornilleria galvanizada
  y el grabado de la placa base (caras a 0.3 mm bajo la cara de arriba).
- Las piezas pesadas se simplifican para que Gazebo vaya fluido.
"""
import os
import pickle
import re
import sys
from collections import defaultdict

import fast_simplification
import numpy as np
import trimesh

GRUPOS = {'base_fija:1': ('base_link', (0.0, 0.0, 0.0), False),
          'eslabon_1:1': ('link_1', (0.0, 0.0, 114.0), True),
          'eslabon_2:1': ('link_2', (160.0, 0.0, 156.0), True),
          'efector_d3:1': ('link_3', (300.0, 0.0, 120.0), True),
          'pinon:1': ('link_3', (300.0, 0.0, 120.0), True),
          'dedo_pos:1': ('dedo_1', (300.0, 0.0, 120.0), True),
          'dedo_neg:1': ('dedo_2', (300.0, 0.0, 120.0), True)}
# Triangulos por eslabon, aproximados, despues de simplificar.
PRESUPUESTO = {'base_link': 70000, 'link_1': 40000, 'link_2': 40000, 'link_3': 25000,
               'dedo_1': 4000, 'dedo_2': 4000}

BLANCO_FUSION = (0.9216, 0.9216, 0.8963)          # el STEP pone este a acrilico y PLA
PLA = ('cubo_theta1', 'cubo_codo', 'leva_theta1', 'brazo_topes_theta2', 'mordaza_cuerpo',
       'pinon_m1_z18', 'carro_dedo_pos', 'carro_dedo_neg')
ACRILICO = (0.82, 0.91, 0.96, 0.35)
GRABADO = (0.10, 0.10, 0.11, 1.0)
GALVANIZADO = (0.72, 0.73, 0.75, 1.0)
# Piezas con mucho detalle que casi no se ve: se simplifican mas.
POCO_VISIBLE = ('pcb_copper', 'pcb_via', 'SILKSCREEN', 'SOLDERMASK', 'ATMEL', 'HIROSE', 'FCI_', 'Dielectric',
                'husillo_Tr8', 'Rodamiento', 'MURATA', 'CAPC', 'RESC', 'SOT', 'OSC_', 'AP3445', 'BOURNS',
                '2023_06_14', 'Polea 20T')
# Cuerpos que no se ven desde fuera (dentro de la PCB o bajo la placa de la camara).
OCULTAS = ('pcb_via', 'SILKSCREEN_BOTTOM', 'SOLDERMASK_BOTTOM')
# Capas finas de las placas (cobre, serigrafia, mascara).
CAPAS = ('pcb_copper', 'SILKSCREEN_TOP', 'SOLDERMASK_TOP')
# Piezas que se ven mucho: se simplifican poco.
MUY_VISIBLE = ('37d-gearmotor', 'SG90', 'Polea 60T')
ZINCADOS = ('DIN 125', 'DIN 934', 'DIN 985', 'DIN 439', 'DIN 9021', 'ISO 7380', 'ISO 10642')


def color_de(pieza, col):
    base = re.sub(r':\d+$', '', pieza.split('#')[0])
    rgb = col[:3]
    if any(base.startswith(z) for z in ZINCADOS):
        return GALVANIZADO
    if rgb == BLANCO_FUSION:
        return (0.93, 0.93, 0.91, 1.0) if base in PLA else ACRILICO
    return col


def simplificar(tri, objetivo):
    """tri: Nx3x3 -> (V, F) indexado y simplificado a ~objetivo triangulos."""
    v, inv = np.unique(np.round(tri.reshape(-1, 3), 6), axis=0, return_inverse=True)
    f = inv.reshape(-1, 3)
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    if objetivo < len(f) and len(f) > 400:
        red = 1.0 - objetivo / len(f)
        v, f = fast_simplification.simplify(v.astype(np.float32), f.astype(np.int32), target_reduction=red)
    return np.asarray(v, float), np.asarray(f, int)


def escribir_dae(ruta, partes, titulo):
    """partes: lista de (rgba, V, F, N) -> Collada 1.4.1 con un material por color."""
    mats, geos, insts = [], [], []
    for k, (rgba, V, F, N) in enumerate(partes):
        r, g, b, a = rgba
        if a < 1.0:
            a = 1.0                           # la transparencia la pone el URDF
        mats.append(f'''    <effect id="e{k}"><profile_COMMON><technique sid="common"><phong>
      <diffuse><color>{r:.4f} {g:.4f} {b:.4f} {a:.3f}</color></diffuse>
      <specular><color>0.25 0.25 0.25 1</color></specular><shininess><float>40</float></shininess>
    </phong></technique></profile_COMMON></effect>''')
        pos = ' '.join(f'{x:.5g}' for x in V.ravel())
        nor = ' '.join(f'{x:.3g}' for x in N.ravel())
        idx = ' '.join(str(i) for i in F.ravel())
        geos.append(f'''    <geometry id="g{k}"><mesh>
      <source id="g{k}p"><float_array id="g{k}pa" count="{V.size}">{pos}</float_array>
        <technique_common><accessor source="#g{k}pa" count="{len(V)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
      <source id="g{k}n"><float_array id="g{k}na" count="{N.size}">{nor}</float_array>
        <technique_common><accessor source="#g{k}na" count="{len(N)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
      <vertices id="g{k}v"><input semantic="POSITION" source="#g{k}p"/><input semantic="NORMAL" source="#g{k}n"/></vertices>
      <triangles material="m{k}" count="{len(F)}"><input semantic="VERTEX" source="#g{k}v" offset="0"/><p>{idx}</p></triangles>
    </mesh></geometry>''')
        insts.append(f'''      <node id="n{k}"><instance_geometry url="#g{k}"><bind_material><technique_common>
        <instance_material symbol="m{k}" target="#m{k}"/></technique_common></bind_material></instance_geometry></node>''')
    materiales = '\n'.join(f'    <material id="m{k}"><instance_effect url="#e{k}"/></material>' for k in range(len(partes)))
    xml = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><contributor><authoring_tool>{titulo}</authoring_tool></contributor>
    <unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_effects>
{chr(10).join(mats)}
  </library_effects>
  <library_materials>
{materiales}
  </library_materials>
  <library_geometries>
{chr(10).join(geos)}
  </library_geometries>
  <library_visual_scenes><visual_scene id="escena">
{chr(10).join(insts)}
  </visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#escena"/></scene>
</COLLADA>
'''
    open(ruta, 'w').write(xml)


def main():
    datos = pickle.load(open(sys.argv[1], 'rb'))['mallas']
    sal = sys.argv[2]
    os.makedirs(sal, exist_ok=True)
    # 1. a pose cero y al marco de cada eslabon, en metros; color final
    por_link = defaultdict(list)           # link -> [(pieza, rgba, tri)]
    # plano medio de la PCB de control: el cobre de debajo no se ve
    pcb = [t for (p, c), t in datos['base_fija:1'].items() if p.startswith('pcb_PCB')]
    z_pcb = np.concatenate(pcb)[:, :, 2].mean() if pcb else None
    for g, piezas in datos.items():
        link, (ox, oy, oz), girar = GRUPOS[g]
        for (pieza, col), tri in piezas.items():
            if any(pieza.startswith(x) for x in OCULTAS):
                continue
            t = tri.copy()
            if pieza.startswith('pcb_copper') and z_pcb is not None:
                t = t[t[:, :, 2].mean(1) > z_pcb]
            if any(pieza.startswith(x) for x in CAPAS):
                # capa de 35 um: solo la cara de fuera (sin paredes ni la cara
                # pegada a la placa, que no se ven)
                n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
                n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
                zc = t[:, :, 2].mean(1)
                arriba = (n[:, 2] > 0.9) & (zc > zc.max() - 0.01)
                abajo = (n[:, 2] < -0.9) & (zc < zc.min() + 0.01)
                fuera = arriba if pieza.startswith('pcb_copper') else (arriba | abajo)
                t = t[fuera]
            if girar:                       # reposo (theta1 = 90) -> pose cero
                t = np.stack([t[..., 1], -t[..., 0], t[..., 2]], axis=-1)
            t = (t - np.array([ox, oy, oz])) / 1000.0
            rgba = color_de(pieza, col)
            if pieza.startswith('placa_base'):
                zc = t[:, :, 2].mean(1)
                gr = (zc > 0.01165) & (zc < 0.01198)
                por_link[link].append((pieza + '/grabado', GRABADO, t[gr]))
                t = t[~gr]
            por_link[link].append((pieza, rgba, t))
    # 2. simplificar por pieza, en proporcion al presupuesto del eslabon
    for link, lista in por_link.items():
        total = sum(len(t) for _, _, t in lista)

        def objetivo(pieza, n, k):
            return n                         # sin simplificar: deformaba las piezas curvas
            if pieza.endswith('/grabado') or pieza.startswith('placa_base') or n < 200:
                return n
            if any(x in pieza for x in MUY_VISIBLE):
                return min(n, max(100, int(n * 0.6)))
            w = 0.2 if any(x in pieza for x in POCO_VISIBLE) else 1.0
            return min(n, max(100, int(n * k * w)))
        k0, k1 = 0.0, 5.0                    # biseccion: k tal que el total cumpla el presupuesto
        for _ in range(40):
            k = (k0 + k1) / 2
            if sum(objetivo(p, len(t), k) for p, _, t in lista) > PRESUPUESTO[link]:
                k1 = k
            else:
                k0 = k
        k = k0
        acum = defaultdict(list)             # rgba -> [(V, F)]
        for pieza, rgba, t in lista:
            if len(t) == 0:
                continue
            obj = objetivo(pieza, len(t), k)
            V, F = simplificar(t, obj)
            if len(F):
                acum[rgba].append((V, F))
        partes, acr, ntri = [], [], 0
        for rgba, vfs in acum.items():
            V = np.concatenate([v for v, _ in vfs])
            off = np.cumsum([0] + [len(v) for v, _ in vfs[:-1]])
            F = np.concatenate([f + o for (_, f), o in zip(vfs, off)])
            m = trimesh.Trimesh(V, F, process=False)
            m = m.smooth_shaded if len(F) < 400000 else m   # normales suaves con corte en las aristas
            parte = (rgba, np.asarray(m.vertices), np.asarray(m.faces), np.asarray(m.vertex_normals))
            (acr if rgba == ACRILICO else partes).append(parte)
            ntri += len(F)
        ruta = os.path.join(sal, link + '.dae')
        escribir_dae(ruta, partes, 'station2 TRIADA, CAD v5.7 (STEP), metros, pose cero')
        if acr:
            # el acrilico va aparte: RViz no lee la opacidad de la malla, y el
            # URDF le pone su color transparente
            # STL sin material propio: RViz aplica entonces el color del URDF
            # con su transparencia (con Collada la ignora)
            V = np.concatenate([p[1] for p in acr])
            off = np.cumsum([0] + [len(p[1]) for p in acr[:-1]])
            F = np.concatenate([p[2] + o for p, o in zip(acr, off)])
            trimesh.Trimesh(V, F, process=False).export(os.path.join(sal, link + '_acrilico.stl'))
        todo = np.concatenate([p[1] for p in partes])
        print(f'{link:9s} {total:7d} -> {ntri:6d} tri, {len(partes):3d} materiales, '
              f'{os.path.getsize(ruta)/1e6:5.2f} MB, x {todo[:,0].min():.4f}..{todo[:,0].max():.4f} '
              f'y {todo[:,1].min():.4f}..{todo[:,1].max():.4f} z {todo[:,2].min():.4f}..{todo[:,2].max():.4f}')


main()
