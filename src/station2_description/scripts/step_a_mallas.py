"""STEP del ensamble (con colores) -> triangulos por grupo de primer nivel y color.

Uso: python step_a_mallas.py <step> <salida.pkl> [deflexion_mm]
Guarda {grupo: {(r,g,b,a): array Nx3x3 en mm}} y un resumen por pieza.
"""
import pickle
import sys
from collections import defaultdict

import numpy as np
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.RWMesh import RWMesh_FaceIterator
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.XCAFPrs import XCAFPrs_DocumentExplorer, XCAFPrs_DocumentExplorerFlags_None

paso, salida = sys.argv[1], sys.argv[2]
defl = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15

doc = TDocStd_Document(TCollection_ExtendedString('doc'))
r = STEPCAFControl_Reader()
r.SetColorMode(True)
r.SetNameMode(True)
assert r.ReadFile(paso) == IFSelect_RetDone
r.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def nombre(l):
    n = TDataStd_Name()
    return n.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), n) else '?'


datos = defaultdict(lambda: defaultdict(list))
piezas = defaultdict(lambda: defaultdict(int))
exp = XCAFPrs_DocumentExplorer(doc, XCAFPrs_DocumentExplorerFlags_None)
nodos = 0
while exp.More():
    nodo = exp.Current()
    prof = exp.CurrentDepth()
    if prof >= 1:
        lab = nodo.RefLabel
        if not st.IsAssembly_s(lab):
            # grupo = componente de primer nivel (profundidad 1) de la ruta
            g = nombre(exp.Current(1).Label)
            pieza = nombre(nodo.Label)
            forma = st.GetShape_s(lab)
            BRepMesh_IncrementalMesh(forma, defl, False, 0.4, True)
            it = RWMesh_FaceIterator(lab, nodo.Location, True, nodo.Style)
            while it.More():
                if it.IsEmptyMesh():
                    it.Next()
                    continue
                c = it.FaceColor()
                rgb = c.GetRGB()
                col = (round(rgb.Red(), 4), round(rgb.Green(), 4), round(rgb.Blue(), 4), round(c.Alpha(), 3))
                n0 = it.NodeLower()
                n = it.NodeUpper() - n0 + 1
                P = np.empty((n, 3))
                for i in range(n):
                    p = it.NodeTransformed(n0 + i)
                    P[i] = (p.X(), p.Y(), p.Z())
                T = []
                for i in range(it.ElemLower(), it.ElemUpper() + 1):
                    a, b, cc = it.TriangleOriented(i).Get()
                    T.append((a - n0, b - n0, cc - n0))
                if T:
                    datos[g][(pieza, col)].append(P[np.array(T)])
                    piezas[g][(pieza, col)] += len(T)
                it.Next()
            nodos += 1
    exp.Next()

out = {g: {c: np.concatenate(v) for c, v in d.items()} for g, d in datos.items()}
pickle.dump({'mallas': out, 'piezas': {g: dict(d) for g, d in piezas.items()}}, open(salida, 'wb'))
for g, d in out.items():
    todo = np.concatenate(list(d.values())).reshape(-1, 3)
    print(f'{g:20s} {sum(len(v) for v in d.values()):8d} tri {len(d):3d} piezas-color '
          f'x {todo[:,0].min():7.1f}..{todo[:,0].max():7.1f} y {todo[:,1].min():7.1f}..{todo[:,1].max():7.1f} '
          f'z {todo[:,2].min():6.1f}..{todo[:,2].max():6.1f}')
print('nodos hoja', nodos)
