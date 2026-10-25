"""Hoja de contacto: un primer plano de cada pieza distinta del STEP teselado.

Uso: python hoja_contacto.py <mallas.pkl> <prefijo_salida>
"""
import pickle
import re
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

d = pickle.load(open(sys.argv[1], 'rb'))['mallas']
piezas = {}
for g, m in d.items():
    por = defaultdict(list)
    for (p, c), t in m.items():
        por[p].append((c, t))
    for p, lst in por.items():
        base = re.sub(r':\d+$', '', p.split('#')[0])
        n = sum(len(t) for _, t in lst)
        if base not in piezas or n > piezas[base][0]:
            piezas[base] = (n, lst)
nombres = sorted(piezas)
por_hoja = 48
luz = np.array([0.4, -0.5, 0.75]); luz /= np.linalg.norm(luz)
for h in range(0, len(nombres), por_hoja):
    fig = plt.figure(figsize=(24, 18))
    for i, nom in enumerate(nombres[h:h + por_hoja]):
        ax = fig.add_subplot(6, 8, i + 1, projection='3d')
        tri_all, cols = [], []
        for c, t in piezas[nom][1]:
            nrm = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
            nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
            k = 0.35 + 0.65 * np.abs(nrm @ luz)
            base = np.array(c[:3]) * 0.85 + 0.1
            cols.append(np.clip(base[None, :] * k[:, None], 0, 1))
            tri_all.append(t)
        T = np.concatenate(tri_all)
        C = np.concatenate(cols)
        ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolors='none', linewidths=0))
        lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
        c0, r = (lo + hi) / 2, (hi - lo).max() / 2 * 1.05 + 1e-6
        ax.set_xlim(c0[0] - r, c0[0] + r); ax.set_ylim(c0[1] - r, c0[1] + r); ax.set_zlim(c0[2] - r, c0[2] + r)
        ax.view_init(25, -60)
        ax.set_axis_off()
        ax.set_title(f'{nom[:28]}\n{len(T)} tri', fontsize=7)
    plt.tight_layout()
    plt.savefig(f'{sys.argv[2]}_{h // por_hoja + 1}.png', dpi=60)
    plt.close(fig)
print(len(nombres), 'piezas distintas')
