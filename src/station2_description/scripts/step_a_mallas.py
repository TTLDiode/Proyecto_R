"""STEP del ensamble (con colores) -> triangulos por grupo de primer nivel y color.

Uso: python step_a_mallas.py <step> <salida.pkl> [deflexion_mm] [deflexion_angular_rad]
Guarda {grupo: {(r,g,b,a): array Nx3x3 en mm}} y un resumen por pieza.
"""
import pickle
import sys
from collections import defaultdict

import numpy as np
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopLoc import TopLoc_Location
from OCP.IFSelect import IFSelect_RetDone
from OCP.RWMesh import RWMesh_FaceIterator
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.XCAFPrs import XCAFPrs_DocumentExplorer, XCAFPrs_DocumentExplorerFlags_None

GRUESO = ('pcb_copper', 'SILKSCREEN', 'SOLDERMASK', 'ATMEL', 'HIROSE', 'FCI_', 'MURATA', 'BOURNS',
          'CAPC', 'RESC', 'SOT', 'OSC_', 'AP3445', 'Dielectric')
paso, salida = sys.argv[1], sys.argv[2]
defl = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15
ang = float(sys.argv[4]) if len(sys.argv) > 4 else 0.4

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
props = {}
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
            pieza = nombre(nodo.Label) + '#' + nodo.Id.ToCString()
            forma = st.GetShape_s(lab)
            # detalle plano o diminuto que casi no se ve: teselado mas grueso
            # (los arcos quedan algo mas poligonales; la forma no cambia)
            if any(x in pieza for x in GRUESO):
                BRepMesh_IncrementalMesh(forma, 1.0, False, 1.0, True)
            else:
                BRepMesh_IncrementalMesh(forma, defl, False, ang, True)
            # propiedades del solido exacto, en el sitio del ensamble
            fl = forma.Moved(nodo.Location)
            gv, ga = GProp_GProps(), GProp_GProps()
            BRepGProp.VolumeProperties_s(fl, gv)
            BRepGProp.SurfaceProperties_s(fl, ga)
            bb = Bnd_Box(); BRepBndLib.Add_s(fl, bb, False)
            c0, c1 = bb.CornerMin(), bb.CornerMax()
            props[(g, pieza)] = (abs(gv.Mass()), ga.Mass(), (c0.X(), c0.Y(), c0.Z(), c1.X(), c1.Y(), c1.Z()))
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
pickle.dump({'mallas': out, 'piezas': {g: dict(d) for g, d in piezas.items()}, 'props': props}, open(salida, 'wb'))
for g, d in out.items():
    todo = np.concatenate(list(d.values())).reshape(-1, 3)
    print(f'{g:20s} {sum(len(v) for v in d.values()):8d} tri {len(d):3d} piezas-color '
          f'x {todo[:,0].min():7.1f}..{todo[:,0].max():7.1f} y {todo[:,1].min():7.1f}..{todo[:,1].max():7.1f} '
          f'z {todo[:,2].min():6.1f}..{todo[:,2].max():6.1f}')
print('nodos hoja', nodos)
