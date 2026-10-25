"""Compara cada cuerpo teselado con su solido exacto del STEP.

Uso: python verificar_mallas.py <mallas.pkl> [tolerancia]
Por cuerpo: area de la malla frente al area del solido, volumen (si la malla
es cerrada), caja envolvente y triangulos muy alargados. Imprime los que se
salen de la tolerancia (2 % por defecto) y un resumen.
"""
import pickle
import sys
from collections import defaultdict

import numpy as np

d = pickle.load(open(sys.argv[1], 'rb'))
tol = float(sys.argv[2]) if len(sys.argv) > 2 else 0.02
props = d['props']
tri = defaultdict(list)
for g, m in d['mallas'].items():
    for (pieza, col), t in m.items():
        tri[(g, pieza)].append(t)

malos, buenos, info = [], 0, []
for clave, (vol, area, bb) in sorted(props.items()):
    if clave not in tri:
        info.append((clave, 'sin malla'))
        continue
    t = np.concatenate(tri[clave])
    e1, e2 = t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]
    cr = np.cross(e1, e2)
    a_m = 0.5 * np.linalg.norm(cr, axis=1).sum()
    v_m = abs(np.einsum('ij,ij->i', t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6)
    lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
    bb0, bb1 = np.array(bb[:3]), np.array(bb[3:])
    tam = max(np.max(bb1 - bb0), 1e-9)
    d_bb = max(np.max(np.abs(lo - bb0)), np.max(np.abs(hi - bb1))) / tam
    d_a = abs(a_m - area) / max(area, 1e-9)
    d_v = abs(v_m - vol) / max(vol, 1e-9) if vol > 1e-6 else 0.0
    # triangulos en punta: lado mayor / altura minima
    l = np.stack([np.linalg.norm(e1, axis=1), np.linalg.norm(e2, axis=1),
                  np.linalg.norm(t[:, 2] - t[:, 1], axis=1)], 1).max(1)
    h = np.linalg.norm(cr, axis=1) / np.maximum(l, 1e-12)
    largo = ((l / np.maximum(h, 1e-12) > 1e4) & (l > 0.02 * tam)).mean()
    mal = d_a > tol or d_bb > 0.02 or (d_v > tol and d_v < 0.9)
    if mal:
        malos.append((clave, d_a, d_v, d_bb, largo, len(t)))
    else:
        buenos += 1
print(f'cuerpos: {len(props)}  bien: {buenos}  fuera de tolerancia: {len(malos)}  sin malla: {len(info)}')
for (g, p), da, dv, db, lg, n in sorted(malos, key=lambda x: -x[1])[:60]:
    print(f'  {g:14s} {p[:45]:45s} area {da*100:6.2f} %  vol {dv*100:6.2f} %  caja {db*100:5.2f} %  agujas {lg*100:4.1f} %  tri {n}')
for c, m in info[:20]:
    print('  ', c, m)
