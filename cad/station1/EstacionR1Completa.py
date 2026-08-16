# -*- coding: utf-8 -*-
"""
Estacion 1 - Alimentacion y singulacion. Proyecto R, Linea Simulada.

Construye la estacion completa y detallada en un documento nuevo: estructura,
bridas con sus taladros, rodamientos, poleas, conjunto del husillo, efector y
electronica. Sustituye al script EstacionR1, que solo generaba el esqueleto.

Responsable: Santiago Fernando Machado Sanchez

CONFIGURACION RRP, TIPO SCARA, CON TRANSMISION POR CORREA

Los motores Pololu 37D pesan unos 210 g y miden 92 mm, de modo que montarlos
sobre cada articulacion rompia la restriccion de altura y dejaba la masa en
voladizo. La disposicion es la habitual en manipuladores SCARA:

    theta1  motor alojado dentro de la columna, accionamiento directo
    theta2  motor sobre el eslabon 1 y coaxial con theta1, correa al codo
    d3      motor sobre el eslabon 2, correa al husillo

El eje de salida del motor no soporta el brazo: cada articulacion rotacional
gira sobre su propio rodamiento y el motor solo transmite par.

En el eje vertical el husillo se aparta 20 mm del eje de la herramienta, porque
la tuerca ocupa ese sitio, y la varilla de guia antigiro se aparta otros 20 al
lado contrario. El carro cruza los tres, de modo que tool0 permanece en
x = 320, y = 0 y el alcance nominal no cambia.

APILADO VERTICAL

      0 -  12   placa base
     12 - 160   columna, con el motor de theta1 en su interior
    160 - 166   brida del motor de theta1
    166 - 173   rodamiento de theta1
    173 - 207   eslabon 1, estructura hueca con la correa por dentro
    185 - 199   plano de la correa de theta2
    207 - 277   motor de theta2
    173 - 307   husillo, cuyo extremo es el punto mas alto de la estacion

    Limite de la guia 320 mm. Alcance 320 mm, el valor nominal exacto.

Uso: Utilidades > ADD-INS > Scripts, seleccionar EstacionR1Completa y Ejecutar.
"""
import math
import os
import traceback

import adsk.core
import adsk.fusion

# --------------------------------------------------------------------------
# Cotas en milimetros
# --------------------------------------------------------------------------
C = {
    'l1': 180.0, 'l2': 140.0,

    'base_x': 220.0, 'base_y': 220.0, 'base_z': 12.0,
    'columna_diam': 60.0,

    # Motor Pololu 37D con encoder. Las cotas salen del propio STEP:
    # cuerpo de 72.6 mm, eje de salida de 22 mm y diam 6, desplazado 7 mm
    # del eje del cuerpo por la reductora. Los seis M3 a diam 31 van
    # centrados en el cuerpo, no en el eje de salida.
    'motor_diam': 36.8, 'motor_cuerpo': 72.6, 'motor_eje': 22.0,
    'motor_desfase': 7.0,
    'motor_piloto': 15.5, 'motor_bcd': 31.0, 'motor_tornillo': 3.4,

    'brazo_ancho': 50.0, 'brazo_espesor': 8.0, 'link1_alto': 34.0,
    # El eslabon 2 lleva en voladizo todo el eje vertical, 695 g. A 8 mm
    # flectaba 0.62 mm en su extremo; a 12 mm baja a 0.18 mm. Cuelga por
    # debajo del eslabon 1, de modo que no afecta a la altura total.
    'link2_espesor': 12.0,

    'rod_art_diam': 32.0, 'rod_art_alto': 7.0,     # 6804, 20 x 32 x 7
    'rod_hus_diam': 22.0, 'rod_hus_alto': 7.0,     # 608ZZ, 8 x 22 x 7
    'polea_diam': 16.0, 'polea_alto': 14.0,        # GT2 de 20 dientes
    'brida_espesor': 6.0,

    'husillo_diam': 8.0, 'guia_diam': 8.0, 'y_husillo': 20.0, 'y_guia': -20.0,
    'tuerca_diam': 15.0, 'tuerca_alto': 15.0,
    'casquillo_diam': 12.0, 'casquillo_alto': 12.0,
    'carro_espesor': 10.0, 'vastago_diam': 16.0, 'vastago_largo': 115.0,
    'gripper_x': 40.0, 'gripper_y': 30.0, 'gripper_z': 35.0,
    'carrera_d3': 80.0,

    'pico_x': 51.0, 'pico_y': 21.0, 'pico_z': 4.0,
    'driver_x': 20.0, 'driver_y': 20.0, 'driver_z': 20.0,
    'fin_x': 43.0, 'fin_y': 28.0, 'fin_z': 21.0,
}

# Modelos del fabricante. Se importan y se colocan sobre la marcha; si alguno
# falla, la pieza se genera como envolvente primitiva y queda anotado.
IMPORTAR_REALES = True
CARPETA_CAD = os.path.expanduser('~/Documents/proyecto-r-linea-simulada/cad')

MODELOS = {
    'motor': 'Gearmotor_37D_100.stp',
    'pico':  'raspberry pi pico/raspberry pi pico.step',
    'driver': 'doble-puente-h-tb6612fng-1.snapshot.1/DOBLE PUENTE H.step',
    'fin':   'kw3-oz-switch-fin-de-carrera-1.snapshot.4/KW3-OZ.STEP',
}

MATERIALES = {
    'MDF':      ['MDF', 'Medium Density Fiberboard', 'Madera', 'Wood'],
    'ACRILICO': ['Acrylic', 'ABS Plastic', 'Plastic', 'Acrilico'],
    'PLA':      ['PLA', 'ABS Plastic', 'Plastic'],
    'ACERO':    ['Steel', 'Acero', 'Stainless Steel'],
    'ALUMINIO': ['Aluminum', 'Aluminio', 'Aluminum 6061'],
    'LATON':    ['Brass', 'Laton', 'Bronze'],
}


def mm(v):
    """Fusion trabaja internamente en centimetros."""
    return v / 10.0


def buscar_material(app, candidatos):
    for libreria in app.materialLibraries:
        for nombre in candidatos:
            m = libreria.materials.itemByName(nombre)
            if m:
                return m
    return None


def aplicar_material(app, design, comp, clave, avisos):
    material = buscar_material(app, MATERIALES[clave])
    if not material:
        avisos.append('Sin material para {}'.format(clave))
        return
    for cuerpo in comp.bRepBodies:
        try:
            cuerpo.material = material
        except Exception:
            try:
                local = design.materials.itemByName(material.name)
                if not local:
                    local = design.materials.addByCopy(material, material.name)
                cuerpo.material = local
            except Exception:
                avisos.append('No se pudo asignar {} a {}'.format(clave, comp.name))


def design_de(app):
    return adsk.fusion.Design.cast(app.activeProduct)


def _mover(occ, matriz):
    """Aplica una transformacion a una ocurrencia, por cualquiera de las dos vias."""
    try:
        occ.transform2 = matriz
    except Exception:
        occ.transform = matriz


def _consolidar(design):
    """Captura la instantanea sin la cual el desplazamiento no persiste."""
    try:
        if design.snapshots.hasPendingSnapshot:
            design.snapshots.add()
    except Exception:
        pass


def importar_y_colocar(app, raiz, clave, nombre, cx, cy, cz, vertical, avisos):
    """Importa un modelo del fabricante y lo centra en (cx, cy, cz).

    La orientacion interna de cada archivo es desconocida, de modo que el eje
    largo de su envolvente se toma como eje de la pieza. Devuelve True si lo
    consiguio; si no, quien llama genera la envolvente primitiva.
    """
    if not IMPORTAR_REALES:
        return False
    ruta = os.path.join(CARPETA_CAD, MODELOS.get(clave, ''))
    if not os.path.exists(ruta):
        avisos.append('{}: no se encontro el modelo'.format(nombre))
        return False
    try:
        gestor = app.importManager
        antes = raiz.occurrences.count
        opciones = gestor.createSTEPImportOptions(ruta)
        opciones.isViewFit = False
        gestor.importToTarget(opciones, raiz)
        nuevas = [raiz.occurrences.item(i)
                  for i in range(antes, raiz.occurrences.count)]
        if not nuevas:
            avisos.append('{}: la importacion no creo componentes'.format(nombre))
            return False

        xs, ys, zs = [], [], []
        for occ in nuevas:
            bb = occ.boundingBox
            xs += [bb.minPoint.x, bb.maxPoint.x]
            ys += [bb.minPoint.y, bb.maxPoint.y]
            zs += [bb.minPoint.z, bb.maxPoint.z]
        centro = [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2,
                  (min(zs) + max(zs)) / 2]
        dims = [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)]

        matriz = adsk.core.Matrix3D.create()
        largo = dims.index(max(dims))
        if vertical and largo != 2:
            origen = adsk.core.Point3D.create(0, 0, 0)
            if largo == 0:
                matriz.setToRotation(-math.pi / 2,
                                     adsk.core.Vector3D.create(0, 1, 0), origen)
                girado = [-centro[2], centro[1], centro[0]]
            else:
                matriz.setToRotation(math.pi / 2,
                                     adsk.core.Vector3D.create(1, 0, 0), origen)
                girado = [centro[0], -centro[2], centro[1]]
        else:
            girado = list(centro)

        matriz.translation = adsk.core.Vector3D.create(
            mm(cx) - girado[0], mm(cy) - girado[1], mm(cz) - girado[2])

        for i, occ in enumerate(nuevas):
            occ.component.name = (nombre if len(nuevas) == 1
                                  else '{}_{}'.format(nombre, i + 1))
            _mover(occ, matriz)
        _consolidar(design_de(app))

        # Segunda pasada. En un diseno parametrico el desplazamiento de una
        # ocurrencia no se consolida hasta capturar una instantanea, y la
        # envolvente que se leyo antes puede no reflejar lo que Fusion hizo al
        # importar. Se mide donde acabo de verdad y se corrige la diferencia.
        xs, ys, zs = [], [], []
        for occ in nuevas:
            bb = occ.boundingBox
            xs += [bb.minPoint.x, bb.maxPoint.x]
            ys += [bb.minPoint.y, bb.maxPoint.y]
            zs += [bb.minPoint.z, bb.maxPoint.z]
        real = [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2,
                (min(zs) + max(zs)) / 2]
        error = [mm(cx) - real[0], mm(cy) - real[1], mm(cz) - real[2]]

        if max(abs(e) for e in error) > 0.01:
            for occ in nuevas:
                correccion = occ.transform.copy()
                t = correccion.translation
                correccion.translation = adsk.core.Vector3D.create(
                    t.x + error[0], t.y + error[1], t.z + error[2])
                _mover(occ, correccion)
            _consolidar(design_de(app))
        return True
    except Exception as e:
        avisos.append('{}: fallo la importacion ({})'.format(nombre, e))
        return False


# Marco del archivo KW3-OZ.STEP, medido sobre el propio fichero. La
# envolvente completa mide 42.9 x 28.2 x 21.2, pero incluye la palanca. El
# cuerpo, que es lo que se atornilla, es bastante mas pequeno.
FIN_CUERPO_X = (-14.0, 14.6)
FIN_CUERPO_Y = (-12.0, 6.8)
FIN_CUERPO_Z = (-5.0, 5.0)
FIN_PALANCA = 28.93          # alcance de la punta de la palanca, en +x
FIN_RODILLO = 16.22          # altura del rodillo, en +y, que es lo que topa

# Separacion entre el centro del cuerpo y el rodillo. Es la cota que manda al
# situar el interruptor: el cuerpo puede quedar donde sea, pero el rodillo
# tiene que caer dentro de la banda que barre la leva.
FIN_RODILLO_OFS = FIN_RODILLO - (FIN_CUERPO_Y[0] + FIN_CUERPO_Y[1]) / 2.0


def colocar_fin(app, raiz, nombre, centro, u_palanca, u_acciona, avisos):
    """Coloca un KW3-OZ por el centro de su CUERPO, no de su envolvente.

    u_palanca es hacia donde sale la palanca y u_acciona la direccion en que
    asoma el rodillo, que es por donde la leva lo empuja. En el archivo son
    +x e +y, y han de ser perpendiculares. La cara de montaje es la
    perpendicular a ambas. Devuelve True si lo consiguio.
    """
    if not IMPORTAR_REALES:
        return False
    ruta = os.path.join(CARPETA_CAD, MODELOS.get('fin', ''))
    if not os.path.exists(ruta):
        avisos.append('{}: no se encontro el modelo'.format(nombre))
        return False
    try:
        px, py, pz = u_palanca
        qx, qy, qz = u_acciona
        # El eje z del archivo, normal a la cara de montaje, sale del producto
        # vectorial de los otros dos.
        zx = py * qz - pz * qy
        zy = pz * qx - px * qz
        zz = px * qy - py * qx
        rot = [[px, qx, zx], [py, qy, zy], [pz, qz, zz]]

        gestor = app.importManager
        antes = raiz.occurrences.count
        opciones = gestor.createSTEPImportOptions(ruta)
        opciones.isViewFit = False
        gestor.importToTarget(opciones, raiz)
        nuevas = [raiz.occurrences.item(i)
                  for i in range(antes, raiz.occurrences.count)]
        if not nuevas:
            avisos.append('{}: la importacion no creo componentes'.format(nombre))
            return False

        p = [sum(FIN_CUERPO_X) / 2.0, sum(FIN_CUERPO_Y) / 2.0,
             sum(FIN_CUERPO_Z) / 2.0]
        img = [sum(rot[i][j] * p[j] for j in range(3)) for i in range(3)]
        tras = [mm(centro[i] - img[i]) for i in range(3)]

        matriz = adsk.core.Matrix3D.create()
        matriz.setWithArray([
            rot[0][0], rot[0][1], rot[0][2], tras[0],
            rot[1][0], rot[1][1], rot[1][2], tras[1],
            rot[2][0], rot[2][1], rot[2][2], tras[2],
            0.0, 0.0, 0.0, 1.0])

        for i, occ in enumerate(nuevas):
            occ.component.name = (nombre if len(nuevas) == 1
                                  else '{}_{}'.format(nombre, i + 1))
            _mover(occ, matriz)
        _consolidar(design_de(app))
        return True
    except Exception as e:
        avisos.append('{}: fallo la importacion ({})'.format(nombre, e))
        return False


def centro_cuerpo(jx, jy, cuerpo_hacia):
    """Donde cae el eje del cuerpo del motor, y con que fase sus tornillos.

    El eje de salida del 37D esta desplazado 7 mm del eje del cuerpo, y la
    circunferencia de seis M3 va centrada en el cuerpo. Las placas que lo
    sostienen se dibujan antes de importarlo, de modo que necesitan este
    dato por adelantado. Devuelve (cx, cy, fase).
    """
    ux, uy = cuerpo_hacia
    norma = math.hypot(ux, uy) or 1.0
    ux, uy = ux / norma, uy / norma
    return (jx + C['motor_desfase'] * ux,
            jy + C['motor_desfase'] * uy,
            math.atan2(-ux, uy))


def colocar_motor(app, raiz, nombre, jx, jy, z_cara, eje_arriba,
                  cuerpo_hacia, avisos):
    """Coloca un 37D con su eje de salida en (jx, jy) y la cara en z_cara.

    El archivo del fabricante tiene el origen en el plano de la cara de
    montaje y sobre el eje del cuerpo, con el eje de salida hacia +X y
    desplazado 7 mm. Conocido el marco, la matriz se compone en vez de
    deducirse de la envolvente: girar el eje de salida hacia arriba o hacia
    abajo, girar despues alrededor de ese eje para llevar el cuerpo hacia
    donde interese, y trasladar el eje de salida a su sitio.

    cuerpo_hacia es la direccion (ux, uy) hacia la que queda el cuerpo.
    Devuelve (cx, cy, fase) del cuerpo, que es donde hay que centrar la
    circunferencia de tornillos del soporte, o None si no pudo importar.
    """
    cx_cuerpo, cy_cuerpo, fase = centro_cuerpo(jx, jy, cuerpo_hacia)

    if not IMPORTAR_REALES:
        return None
    ruta = os.path.join(CARPETA_CAD, MODELOS.get('motor', ''))
    if not os.path.exists(ruta):
        avisos.append('{}: no se encontro el modelo'.format(nombre))
        return None
    try:
        gestor = app.importManager
        antes = raiz.occurrences.count
        opciones = gestor.createSTEPImportOptions(ruta)
        opciones.isViewFit = False
        gestor.importToTarget(opciones, raiz)
        nuevas = [raiz.occurrences.item(i)
                  for i in range(antes, raiz.occurrences.count)]
        if not nuevas:
            avisos.append('{}: la importacion no creo componentes'.format(nombre))
            return None

        # Alineacion del eje de salida, que en el archivo apunta hacia +X.
        if eje_arriba:
            alinear = [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]
        else:
            alinear = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]
        co, si = math.cos(fase), math.sin(fase)
        giro = [[co, -si, 0.0], [si, co, 0.0], [0.0, 0.0, 1.0]]
        rot = [[sum(giro[i][k] * alinear[k][j] for k in range(3))
                for j in range(3)] for i in range(3)]

        # El eje de salida ocupa (0, -desfase, 0) en el archivo. Se lleva a
        # (jx, jy, z_cara), y el cuerpo cae solo donde toca.
        p = [0.0, -C['motor_desfase'], 0.0]
        img = [sum(rot[i][j] * p[j] for j in range(3)) for i in range(3)]
        destino = [jx, jy, z_cara]
        tras = [mm(destino[i] - img[i]) for i in range(3)]

        matriz = adsk.core.Matrix3D.create()
        matriz.setWithArray([
            rot[0][0], rot[0][1], rot[0][2], tras[0],
            rot[1][0], rot[1][1], rot[1][2], tras[1],
            rot[2][0], rot[2][1], rot[2][2], tras[2],
            0.0, 0.0, 0.0, 1.0])

        for i, occ in enumerate(nuevas):
            occ.component.name = (nombre if len(nuevas) == 1
                                  else '{}_{}'.format(nombre, i + 1))
            _mover(occ, matriz)
        _consolidar(design_de(app))

        # Comprobacion. El cuerpo domina la envolvente y es de revolucion
        # alrededor de su propio eje, de modo que se sabe donde debe caer su
        # centro. Si no cae ahi, se corrige y se avisa, porque con una matriz
        # exacta no deberia hacer falta.
        largo = C['motor_cuerpo'] / 2.0 - C['motor_eje'] / 2.0
        esperado = [mm(cx_cuerpo), mm(cy_cuerpo),
                    mm(z_cara + (-largo if eje_arriba else largo))]
        xs, ys, zs = [], [], []
        for occ in nuevas:
            bb = occ.boundingBox
            xs += [bb.minPoint.x, bb.maxPoint.x]
            ys += [bb.minPoint.y, bb.maxPoint.y]
            zs += [bb.minPoint.z, bb.maxPoint.z]
        real = [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2,
                (min(zs) + max(zs)) / 2]
        error = [esperado[i] - real[i] for i in range(3)]
        if max(abs(e) for e in error) > 0.05:
            for occ in nuevas:
                correccion = occ.transform.copy()
                t = correccion.translation
                correccion.translation = adsk.core.Vector3D.create(
                    t.x + error[0], t.y + error[1], t.z + error[2])
                _mover(occ, correccion)
            _consolidar(design_de(app))
            avisos.append('{}: hubo que corregir {:.1f}, {:.1f}, {:.1f} mm'
                          .format(nombre, *[e * 10 for e in error]))
        return (cx_cuerpo, cy_cuerpo, fase)
    except Exception as e:
        avisos.append('{}: fallo la importacion ({})'.format(nombre, e))
        return None


def nuevo_componente(raiz, nombre):
    occ = raiz.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = nombre
    return occ.component


def extruir(comp, perfil, z0, alto):
    ext = comp.features.extrudeFeatures
    entrada = ext.createInput(
        perfil, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    entrada.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(alto)))
    if abs(z0) > 1e-9:
        entrada.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(mm(z0)))
    return ext.add(entrada)


def perfil_mayor(croquis):
    """El perfil de mayor area es la pieza; los demas son sus taladros."""
    mejor, area = None, -1.0
    for i in range(croquis.profiles.count):
        p = croquis.profiles.item(i)
        try:
            a = p.areaProperties().area
        except Exception:
            continue
        if a > area:
            mejor, area = p, a
    return mejor


def circulo(croquis, x, y, diam):
    croquis.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(mm(x), mm(y), 0), mm(diam / 2.0))


def patron_tornillos(croquis, cx, cy, bcd, diam, n=6, fase=0.0):
    for i in range(n):
        a = 2.0 * math.pi * i / n + fase
        circulo(croquis, cx + bcd / 2.0 * math.cos(a),
                cy + bcd / 2.0 * math.sin(a), diam)


def cilindro(raiz, nombre, x, y, z0, diam, alto):
    comp = nuevo_componente(raiz, nombre)
    croquis = comp.sketches.add(comp.xYConstructionPlane)
    circulo(croquis, x, y, diam)
    extruir(comp, croquis.profiles.item(0), z0, alto)
    return comp


def caja_girada(raiz, nombre, cx, cy, z0, dx, dy, dz, ang):
    """Prisma recto girado alrededor de z, centrado en planta en (cx, cy).

    Hace falta para los soportes de los finales de carrera: el interruptor va
    orientado segun la articulacion que vigila, y su cara de montaje solo
    apoya bien si el soporte esta girado igual.
    """
    comp = nuevo_componente(raiz, nombre)
    croquis = comp.sketches.add(comp.xYConstructionPlane)
    co, si = math.cos(ang), math.sin(ang)
    pts = []
    for u, v in ((-dx / 2, -dy / 2), (dx / 2, -dy / 2),
                 (dx / 2, dy / 2), (-dx / 2, dy / 2)):
        pts.append(adsk.core.Point3D.create(
            mm(cx + u * co - v * si), mm(cy + u * si + v * co), 0))
    lineas = croquis.sketchCurves.sketchLines
    l0 = lineas.addByTwoPoints(pts[0], pts[1])
    l1 = lineas.addByTwoPoints(l0.endSketchPoint, pts[2])
    l2 = lineas.addByTwoPoints(l1.endSketchPoint, pts[3])
    lineas.addByTwoPoints(l2.endSketchPoint, l0.startSketchPoint)
    extruir(comp, croquis.profiles.item(0), z0, dz)
    return comp


def caja(raiz, nombre, x0, y0, z0, dx, dy, dz):
    comp = nuevo_componente(raiz, nombre)
    croquis = comp.sketches.add(comp.xYConstructionPlane)
    croquis.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(mm(x0), mm(y0), 0),
        adsk.core.Point3D.create(mm(x0 + dx), mm(y0 + dy), 0))
    extruir(comp, croquis.profiles.item(0), z0, dz)
    return comp


def disco_taladrado(raiz, nombre, x, y, z0, diam_ext, esp,
                    bore=None, bcd=None, tornillo=None, avisos=None,
                    bcd_centro=None, fase=0.0):
    comp = nuevo_componente(raiz, nombre)
    croquis = comp.sketches.add(comp.xYConstructionPlane)
    circulo(croquis, x, y, diam_ext)
    perfil = None
    try:
        if bore:
            circulo(croquis, x, y, bore)
        if bcd and tornillo:
            # La circunferencia de tornillos va centrada en el cuerpo del
            # motor, que no coincide con el eje de salida ni con el disco.
            cx, cy = bcd_centro if bcd_centro else (x, y)
            patron_tornillos(croquis, cx, cy, bcd, tornillo, fase=fase)
        perfil = perfil_mayor(croquis)
    except Exception:
        if avisos is not None:
            avisos.append('{}: sin taladros, queda macizo'.format(nombre))
    if perfil is None:
        perfil = croquis.profiles.item(0)
    extruir(comp, perfil, z0, esp)
    return comp


def placa_taladrada(raiz, nombre, x0, y0, z0, dx, dy, dz,
                    bores=(), patrones=(), avisos=None):
    comp = nuevo_componente(raiz, nombre)
    croquis = comp.sketches.add(comp.xYConstructionPlane)
    croquis.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(mm(x0), mm(y0), 0),
        adsk.core.Point3D.create(mm(x0 + dx), mm(y0 + dy), 0))
    perfil = None
    try:
        for bx, by, bd in bores:
            circulo(croquis, bx, by, bd)
        for patron in patrones:
            px, py, bcd, td = patron[:4]
            fase = patron[4] if len(patron) > 4 else 0.0
            patron_tornillos(croquis, px, py, bcd, td, fase=fase)
        perfil = perfil_mayor(croquis)
    except Exception:
        if avisos is not None:
            avisos.append('{}: sin taladros, queda maciza'.format(nombre))
    if perfil is None:
        perfil = croquis.profiles.item(0)
    extruir(comp, perfil, z0, dz)
    return comp


# --------------------------------------------------------------------------
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        raiz = design.rootComponent
        avisos = []
        c = C

        # ---------- Alturas del apilado ----------
        z_col_top = 160.0
        z_brida = z_col_top
        z_rod1 = z_brida + c['brida_espesor']                    # 166
        z_l1_inf = z_rod1 + c['rod_art_alto']                    # 173
        z_l1_top = z_l1_inf + c['link1_alto']                    # 207
        z_l1_sup = z_l1_top - c['brazo_espesor']                 # 199
        z_correa = z_l1_inf + 12.0                               # 185
        z_l2 = z_l1_inf - c['link2_espesor']                     # 161
        z_l2_top = z_l1_inf                                      # 173
        z_tope = 307.0

        x_codo = c['l1']
        x_hus = c['l1'] + c['l2']
        yh, yg = c['y_husillo'], c['y_guia']

        # ---------- Base y columna ----------
        base = caja(raiz, 'base_link', -c['base_x'] / 2, -c['base_y'] / 2, 0.0,
                    c['base_x'], c['base_y'], c['base_z'])
        aplicar_material(app, design, base, 'MDF', avisos)

        columna = cilindro(raiz, 'columna', 0, 0, c['base_z'],
                           c['columna_diam'], z_col_top - c['base_z'])
        aplicar_material(app, design, columna, 'PLA', avisos)

        # Motor de theta1: eje de salida sobre el eje de la columna y hacia
        # arriba, cara contra la brida y cuerpo colgando dentro de la columna.
        # El cuerpo se desplaza hacia +x; con radio 18.4 y desfase 7 llega a
        # 25.4 del eje, de modo que sigue cabiendo en la columna de diam 60.
        cuerpo1 = centro_cuerpo(0.0, 0.0, (1.0, 0.0))
        if colocar_motor(app, raiz, 'motor_theta1', 0.0, 0.0, z_col_top,
                         True, (1.0, 0.0), avisos) is None:
            m1 = cilindro(raiz, 'motor_theta1', cuerpo1[0], cuerpo1[1],
                          z_col_top - c['motor_cuerpo'],
                          c['motor_diam'], c['motor_cuerpo'])
            aplicar_material(app, design, m1, 'ALUMINIO', avisos)

        brida = disco_taladrado(raiz, 'brida_theta1', 0, 0, z_brida,
                                c['columna_diam'], c['brida_espesor'],
                                bore=c['motor_piloto'], bcd=c['motor_bcd'],
                                tornillo=c['motor_tornillo'], avisos=avisos,
                                bcd_centro=(cuerpo1[0], cuerpo1[1]),
                                fase=cuerpo1[2])
        aplicar_material(app, design, brida, 'ALUMINIO', avisos)

        r1 = cilindro(raiz, 'rodamiento_theta1_6804', 0, 0, z_rod1,
                      c['rod_art_diam'], c['rod_art_alto'])
        aplicar_material(app, design, r1, 'ACERO', avisos)

        # ---------- Eslabon 1, estructura hueca ----------
        # Las placas se prolongan por detras del eje de theta1. Nacian justo
        # sobre el, de modo que el agujero del piloto y los tornillos de la
        # brida caian medio fuera del material.
        # El motor de theta2 se importa mas abajo, pero su placa se dibuja
        # aqui y necesita saber donde caeran sus tornillos.
        cuerpo2 = centro_cuerpo(0.0, 0.0, (1.0, 0.0))
        cola = c['motor_bcd'] / 2.0 + 10.0
        l1i = placa_taladrada(
            raiz, 'link_1_placa_inferior', -cola, -c['brazo_ancho'] / 2,
            z_l1_inf, c['l1'] + cola, c['brazo_ancho'], c['brazo_espesor'],
            bores=[(0.0, 0.0, c['motor_piloto']),
                   (x_codo, 0.0, c['rod_art_diam'])], avisos=avisos)
        aplicar_material(app, design, l1i, 'ACRILICO', avisos)

        l1s = placa_taladrada(
            raiz, 'link_1_placa_superior', -cola, -c['brazo_ancho'] / 2,
            z_l1_sup, c['l1'] + cola, c['brazo_ancho'], c['brazo_espesor'],
            bores=[(0.0, 0.0, c['motor_piloto']),
                   (x_codo, 0.0, 19.0)],
            patrones=[(cuerpo2[0], cuerpo2[1], c['motor_bcd'],
                       c['motor_tornillo'], cuerpo2[2])],
            avisos=avisos)
        aplicar_material(app, design, l1s, 'ACRILICO', avisos)

        for i, (sx, sy) in enumerate([(45, 18), (45, -18), (150, 18), (150, -18)]):
            sep = cilindro(raiz, 'separador_{}'.format(i + 1), sx, sy,
                           z_l1_inf + c['brazo_espesor'], 10.0,
                           z_l1_sup - z_l1_inf - c['brazo_espesor'])
            aplicar_material(app, design, sep, 'ALUMINIO', avisos)

        pm2 = cilindro(raiz, 'polea_theta2_motor', 0, 0, z_correa,
                       c['polea_diam'], c['polea_alto'])
        aplicar_material(app, design, pm2, 'ALUMINIO', avisos)

        pc2 = cilindro(raiz, 'polea_theta2_codo', x_codo, 0, z_correa,
                       c['polea_diam'], c['polea_alto'])
        aplicar_material(app, design, pc2, 'ALUMINIO', avisos)

        # Motor de theta2: cara apoyada en la placa superior del eslabon 1,
        # cuerpo hacia arriba y eje hacia abajo, hasta la polea alojada entre
        # las dos placas. El cuerpo va hacia +x, sobre el propio eslabon.
        if colocar_motor(app, raiz, 'motor_theta2', 0.0, 0.0, z_l1_top,
                         False, (1.0, 0.0), avisos) is None:
            m2 = cilindro(raiz, 'motor_theta2', cuerpo2[0], cuerpo2[1],
                          z_l1_top, c['motor_diam'], c['motor_cuerpo'])
            aplicar_material(app, design, m2, 'ALUMINIO', avisos)

        # ---------- Eslabon 2 ----------
        l2 = placa_taladrada(
            raiz, 'link_2', x_codo - cola, -c['brazo_ancho'] / 2, z_l2,
            c['l2'] + cola, c['brazo_ancho'], c['link2_espesor'],
            bores=[(x_codo, 0.0, c['rod_art_diam']),
                   (x_hus, yh, c['rod_hus_diam']),
                   (x_hus, yg, c['guia_diam'])], avisos=avisos)
        aplicar_material(app, design, l2, 'ACRILICO', avisos)

        # El rodamiento del codo va alojado en la placa inferior del eslabon
        # 1, que es donde tiene su taladro, y no dentro del eslabon 2.
        r2 = cilindro(raiz, 'rodamiento_theta2_6804', x_codo, 0, z_l1_inf,
                      c['rod_art_diam'], c['rod_art_alto'])
        aplicar_material(app, design, r2, 'ACERO', avisos)

        # Eje del codo. Sube desde el eslabon 2, atraviesa el eslabon 1 y
        # sobresale por su cara superior. Es lo que sostiene la polea de la
        # correa de theta2 y la leva de su final de carrera, que hasta ahora
        # estaban las dos en el aire. Escalonado: diam 20 en el rodamiento y
        # diam 10 para la polea, el rodamiento superior y el buje.
        z_eje_paso = z_correa - 1.0
        eji = cilindro(raiz, 'eje_codo_inferior', x_codo, 0, z_l2,
                       20.0, z_eje_paso - z_l2)
        aplicar_material(app, design, eji, 'ACERO', avisos)

        # La leva de theta2 tiene que quedar a la altura del rodillo de su
        # interruptor, que apoya en la cara superior del eslabon 1. De ahi
        # salen tanto el largo del eje como el alto del buje.
        # El interruptor de theta2 se atornilla de plano sobre un calzo que
        # apoya en la cara superior del eslabon 1, y su rodillo queda a media
        # altura del cuerpo. La leva ha de barrer justo ahi.
        calzo2 = 5.0
        z_leva2 = z_l1_top + calzo2
        z_rodillo2 = z_leva2 + (FIN_CUERPO_Z[1] - FIN_CUERPO_Z[0]) / 2.0
        ejs = cilindro(raiz, 'eje_codo_superior', x_codo, 0, z_eje_paso,
                       10.0, z_leva2 - z_eje_paso)
        aplicar_material(app, design, ejs, 'ACERO', avisos)

        r2s = cilindro(raiz, 'rodamiento_theta2_superior_6800', x_codo, 0,
                       z_l1_sup, 19.0, 5.0)
        aplicar_material(app, design, r2s, 'ACERO', avisos)

        # ---------- Eje vertical ----------
        hus = cilindro(raiz, 'husillo_T8', x_hus, yh, z_l2_top,
                       c['husillo_diam'], z_tope - z_l2_top)
        aplicar_material(app, design, hus, 'ACERO', avisos)

        guia = cilindro(raiz, 'varilla_guia', x_hus, yg, z_l2_top,
                        c['guia_diam'], z_tope - z_l2_top)
        aplicar_material(app, design, guia, 'ACERO', avisos)

        z_rod_inf = z_l2_top
        z_pol_hus = z_rod_inf + c['rod_hus_alto']
        z_rod_sup = z_tope - 12.0
        z_tuerca = z_rod_sup - c['tuerca_alto']
        z_carro = z_tuerca - c['carro_espesor']

        ri = cilindro(raiz, 'rodamiento_husillo_inf_608', x_hus, yh,
                      z_rod_inf, c['rod_hus_diam'], c['rod_hus_alto'])
        aplicar_material(app, design, ri, 'ACERO', avisos)

        ph = cilindro(raiz, 'polea_husillo', x_hus, yh, z_pol_hus,
                      c['polea_diam'], 16.0)
        aplicar_material(app, design, ph, 'ALUMINIO', avisos)

        rs = cilindro(raiz, 'rodamiento_husillo_sup_608', x_hus, yh,
                      z_rod_sup, c['rod_hus_diam'], c['rod_hus_alto'])
        aplicar_material(app, design, rs, 'ACERO', avisos)

        mont = caja(raiz, 'puente_montante', 296.0, 8.0, z_l2_top,
                    10.0, 24.0, z_tope - 8.0 - z_l2_top)
        aplicar_material(app, design, mont, 'ALUMINIO', avisos)

        plato = placa_taladrada(
            raiz, 'puente_plato', 296.0, -34.0, z_tope - 8.0, 38.0, 68.0, 8.0,
            bores=[(x_hus, yh, c['rod_hus_diam']),
                   (x_hus, yg, c['guia_diam'])], avisos=avisos)
        aplicar_material(app, design, plato, 'ALUMINIO', avisos)

        # ---------- Motor de d3 ----------
        # El motor de d3 no puede acercarse al codo. La cara superior del
        # eslabon 2 esta en z = 173, que es exactamente la cara inferior del
        # eslabon 1: cualquier pieza montada encima invade su banda vertical
        # y choca con el al plegar el codo. Con la semianchura del soporte,
        # 23.5 mm, la cuenta a theta2 = 135 grados exige estar a mas de
        # 58.9 mm del codo.
        #
        # La posicion sale por tanto de la correa: una GT2 cerrada de 140 mm
        # con dos poleas de 20 dientes da 50 mm exactos entre ejes, que dejan
        # el conjunto a 63.7 mm del codo. El eje va sobre la linea media del
        # eslabon; en y = 20 la circunferencia de tornillos se salia 10.5 mm
        # por el borde de los 50 mm de ancho del eslabon 2.
        c_correa_d3 = 50.0
        y_mot_d3 = 0.0
        x_mot_d3 = x_hus - math.sqrt(c_correa_d3 ** 2 - (yh - y_mot_d3) ** 2)
        z_cara_d3 = z_pol_hus + c['motor_eje']
        cuerpo3 = centro_cuerpo(x_mot_d3, y_mot_d3, (-1.0, 0.0))

        # El plato se dimensiona a partir de la circunferencia de tornillos
        # ya descentrada, con 8 mm de borde, en vez de suponerla centrada.
        r_bcd = c['motor_bcd'] / 2.0 + 8.0
        sop_x0 = min(x_mot_d3, cuerpo3[0] - r_bcd)
        sop_x1 = max(x_mot_d3, cuerpo3[0] + r_bcd)
        sop = placa_taladrada(
            raiz, 'soporte_d3_plato', sop_x0, y_mot_d3 - r_bcd, z_cara_d3,
            sop_x1 - sop_x0, 2.0 * r_bcd, 6.0,
            bores=[(x_mot_d3, y_mot_d3, c['motor_piloto'])],
            patrones=[(cuerpo3[0], cuerpo3[1], c['motor_bcd'],
                       c['motor_tornillo'], cuerpo3[2])],
            avisos=avisos)
        aplicar_material(app, design, sop, 'ALUMINIO', avisos)

        # Dos montantes, uno en cada borde en y. Quedan fuera de la correa,
        # que sale del motor en y = 0 y solo alcanza y = 20 junto al husillo,
        # y fuera de los tornillos, que llegan a y = +-15.5. El plato deja de
        # volar en el aire y pasa a apoyar en sus dos lados.
        for i, signo in enumerate((-1.0, 1.0)):
            y_mont = y_mot_d3 + signo * r_bcd - (6.0 if signo > 0 else 0.0)
            sopm = caja(raiz, 'soporte_d3_montante_{}'.format(i + 1),
                        sop_x0, y_mont, z_l2_top,
                        sop_x1 - sop_x0, 6.0, z_cara_d3 - z_l2_top)
            aplicar_material(app, design, sopm, 'ALUMINIO', avisos)

        # Cara inferior a 6 mm por encima del plato, eje hacia abajo hasta la
        # polea. El cuerpo queda arriba, despejado.
        if colocar_motor(app, raiz, 'motor_d3', x_mot_d3, y_mot_d3,
                         z_cara_d3 + 6.0, False, (-1.0, 0.0), avisos) is None:
            m3 = cilindro(raiz, 'motor_d3', cuerpo3[0], cuerpo3[1],
                          z_cara_d3 + 6.0, c['motor_diam'], c['motor_cuerpo'])
            aplicar_material(app, design, m3, 'ALUMINIO', avisos)

        pm3 = cilindro(raiz, 'polea_d3_motor', x_mot_d3, y_mot_d3, z_pol_hus,
                       c['polea_diam'], 16.0)
        aplicar_material(app, design, pm3, 'ALUMINIO', avisos)

        # ---------- Carro y efector ----------
        carro = placa_taladrada(
            raiz, 'carro', x_hus - 14.0, -30.0, z_carro, 28.0, 60.0,
            c['carro_espesor'],
            bores=[(x_hus, yh, c['tuerca_diam']),
                   (x_hus, yg, c['casquillo_diam']),
                   (x_hus, 0.0, c['vastago_diam'])], avisos=avisos)
        aplicar_material(app, design, carro, 'ALUMINIO', avisos)

        tue = cilindro(raiz, 'tuerca_T8', x_hus, yh, z_tuerca,
                       c['tuerca_diam'], c['tuerca_alto'])
        aplicar_material(app, design, tue, 'LATON', avisos)

        cas = cilindro(raiz, 'casquillo_guia', x_hus, yg,
                       z_tuerca + (c['tuerca_alto'] - c['casquillo_alto']) / 2,
                       c['casquillo_diam'], c['casquillo_alto'])
        aplicar_material(app, design, cas, 'LATON', avisos)

        vast = cilindro(raiz, 'vastago', x_hus, 0.0,
                        z_carro - c['vastago_largo'],
                        c['vastago_diam'], c['vastago_largo'])
        aplicar_material(app, design, vast, 'ALUMINIO', avisos)

        z_grip = z_carro - c['vastago_largo'] - c['gripper_z']
        grip = caja(raiz, 'gripper_link', x_hus - c['gripper_x'] / 2,
                    -c['gripper_y'] / 2, z_grip,
                    c['gripper_x'], c['gripper_y'], c['gripper_z'])
        aplicar_material(app, design, grip, 'PLA', avisos)

        # ---------- Electronica sobre la placa base ----------
        # El enunciado pide representar microcontrolador, drivers y fin de
        # carrera para validar que la estacion es fisicamente ensamblable.
        if not importar_y_colocar(app, raiz, 'pico', 'raspberry_pi_pico',
                                  -74.5, 72.5, c['base_z'] + c['pico_z'] / 2,
                                  False, avisos):
            pico = caja(raiz, 'raspberry_pi_pico', -100.0, 62.0, c['base_z'],
                        c['pico_x'], c['pico_y'], c['pico_z'])
            aplicar_material(app, design, pico, 'PLA', avisos)

        for i, dx0 in enumerate([40.0, 70.0]):
            nom = 'driver_TB6612FNG_{}'.format(i + 1)
            if not importar_y_colocar(app, raiz, 'driver', nom, dx0, 34.0,
                                      c['base_z'] + c['driver_z'] / 2,
                                      False, avisos):
                drv = caja(raiz, nom, dx0 - c['driver_x'] / 2, 24.0, c['base_z'],
                           c['driver_x'], c['driver_y'], c['driver_z'])
                aplicar_material(app, design, drv, 'PLA', avisos)

        # Tres finales de carrera, uno por eje, para la referencia de origen.
        # Los encoders son incrementales: al arrancar, el robot no sabe donde
        # esta y necesita un punto duro contra el que referenciarse en cada
        # eje. No hacen falta dos por eje, porque una vez referenciado el
        # limite opuesto lo impone el software.
        #
        # Entre la cara superior del eslabon 2 y la inferior del eslabon 1 no
        # hay hueco: las dos estan en z = 173. Un final de carrera solo cabe
        # entonces por encima del eslabon 1, por debajo del eslabon 2, o a mas
        # de 58.9 mm del codo. Los tres se colocan siguiendo esa regla.
        #
        # Cada uno se acciona en un extremo del recorrido, no a mitad de el,
        # que es lo que exige una referencia de origen repetible.
        # El giro de disparo de theta1 se toma en 180 grados y la leva se pone
        # sobre la linea media del eslabon. Con eso el interruptor cae sobre el
        # eje -x y tanto el como su soporte quedan paralelos a los lados de la
        # placa base, en vez de girados un angulo cualquiera.
        home_theta1 = math.radians(180.0)
        home_theta2 = math.radians(135.0)

        semi_p = (FIN_CUERPO_X[1] - FIN_CUERPO_X[0]) / 2.0
        semi_a = (FIN_CUERPO_Y[1] - FIN_CUERPO_Y[0]) / 2.0
        semi_m = (FIN_CUERPO_Z[1] - FIN_CUERPO_Z[0]) / 2.0

        # theta1. La leva cuelga de la cara inferior del eslabon 1 y barre una
        # corona alrededor del eje. El interruptor se atornilla de plano sobre
        # un pilar que sube de la placa base, con la palanca hacia el eje y el
        # rodillo mirando en el sentido de giro.
        # Alcance de la palanca medido desde el centro del cuerpo.
        pal = FIN_PALANCA - (FIN_CUERPO_X[0] + FIN_CUERPO_X[1]) / 2.0

        rad1 = 88.0
        ang1 = math.pi                       # sobre el eje -x, paralelo a la base
        u_pal1 = (-math.cos(ang1), -math.sin(ang1), 0.0)
        u_acc1 = (math.sin(ang1), -math.cos(ang1), 0.0)

        # El rodillo no cae en el eje del interruptor: sale por la palanca y
        # ademas se desvia de lado, de modo que su radio y su angulo son otros.
        # La leva se coloca en ese punto, girado hacia atras el angulo en que
        # ha de dispararse.
        r_rod1 = math.hypot(rad1 - pal, FIN_RODILLO_OFS)
        a_lev1 = ang1 - math.atan2(FIN_RODILLO_OFS, rad1 - pal) - home_theta1
        lev1 = caja(raiz, 'leva_theta1',
                    r_rod1 * math.cos(a_lev1) - 6.0,
                    r_rod1 * math.sin(a_lev1) - 5.0,
                    z_l1_inf - 12.0, 12.0, 10.0, 12.0)
        aplicar_material(app, design, lev1, 'ALUMINIO', avisos)
        # El rodillo queda a media altura del cuerpo, y el cuerpo ha de caer
        # dentro de la banda vertical que barre la leva sin que el pilar la
        # toque: de ahi los 3 mm de holgura por debajo.
        c1 = (rad1 * math.cos(ang1), rad1 * math.sin(ang1),
              z_l1_inf - 12.0 - 3.0 + semi_m)

        pil = caja_girada(raiz, 'soporte_fin_theta1',
                          c1[0], c1[1], c['base_z'],
                          2.0 * semi_p + 2.0, 2.0 * semi_a + 2.0,
                          c1[2] - semi_m - c['base_z'], ang1 + math.pi)
        aplicar_material(app, design, pil, 'ALUMINIO', avisos)

        # theta2. Mismo montaje de plano, sobre un calzo que apoya en la cara
        # superior del eslabon 1. La leva la lleva el eje del codo.
        rad2 = 46.5
        u_pal2 = (1.0, 0.0, 0.0)
        u_acc2 = (0.0, 1.0, 0.0)
        c2 = (x_codo - rad2, 0.0, z_leva2 + semi_m)

        sop2 = caja(raiz, 'soporte_fin_theta2', c2[0] - semi_p,
                    c2[1] - semi_a, z_l1_top,
                    2.0 * semi_p, 2.0 * semi_a, calzo2)
        aplicar_material(app, design, sop2, 'ALUMINIO', avisos)

        # Buje sobre el extremo del eje del codo, taladrado para que el eje lo
        # atraviese, y leva encima. La leva es una barra que sale del buje
        # hasta el radio en que queda el rodillo.
        buje2 = disco_taladrado(raiz, 'buje_leva_theta2', x_codo, 0, z_l1_top,
                                30.0, calzo2, bore=10.0, avisos=avisos)
        aplicar_material(app, design, buje2, 'ALUMINIO', avisos)

        r_rod2 = math.hypot(rad2 - pal, FIN_RODILLO_OFS)
        a_lev2 = (math.pi - math.atan2(FIN_RODILLO_OFS, rad2 - pal)
                  - home_theta2)
        lev2 = caja_girada(raiz, 'leva_theta2',
                           x_codo + (r_rod2 / 2.0) * math.cos(a_lev2),
                           (r_rod2 / 2.0) * math.sin(a_lev2), z_leva2,
                           r_rod2 + 12.0, 12.0,
                           FIN_CUERPO_Z[1] - FIN_CUERPO_Z[0], a_lev2)
        aplicar_material(app, design, lev2, 'ALUMINIO', avisos)

        # d3. Aqui la leva sube, de modo que el accionamiento es vertical y la
        # cara de montaje vertical. El cuerpo queda 9.4 mm por encima del
        # rodillo, y eso no cabe bajo el eslabon 2 en la linea del husillo: la
        # parte alta del cuerpo se meteria dentro del propio eslabon. Se lleva
        # al costado, pasados los 25 mm de su semianchura, donde tiene sitio de
        # sobra. La leva es un brazo solidario del vastago que pasa por encima
        # de la pinza y llega hasta ese costado.
        lev3_z0 = z_carro - c['vastago_largo']
        lev3_x0 = x_hus - 50.0
        lev3_y0 = -38.0
        lev3 = caja(raiz, 'leva_d3', lev3_x0, lev3_y0, lev3_z0,
                    x_hus + 14.0 - lev3_x0, 10.0 - lev3_y0, z_l2 - lev3_z0)
        aplicar_material(app, design, lev3, 'ALUMINIO', avisos)

        u_pal3 = (1.0, 0.0, 0.0)
        u_acc3 = (0.0, 0.0, -1.0)
        c3 = (lev3_x0 - 0.5 - FIN_CUERPO_X[1], -33.0,
              lev3_z0 - 2.0 + FIN_RODILLO_OFS)

        sop3 = caja(raiz, 'soporte_fin_d3', c3[0] - semi_p, c3[1] + semi_m,
                    z_l2, 2.0 * semi_p, 3.0, c3[2] + semi_a - z_l2)
        aplicar_material(app, design, sop3, 'ALUMINIO', avisos)

        finales = [
            ('fin_de_carrera_theta1', c1, u_pal1, u_acc1),
            ('fin_de_carrera_theta2', c2, u_pal2, u_acc2),
            ('fin_de_carrera_d3', c3, u_pal3, u_acc3),
        ]
        for nom, ctr, u_pal, u_acc in finales:
            if not colocar_fin(app, raiz, nom, ctr, u_pal, u_acc, avisos):
                f = caja(raiz, nom, ctr[0] - semi_p, ctr[1] - semi_a,
                         ctr[2] - semi_m, 2.0 * semi_p, 2.0 * semi_a,
                         2.0 * semi_m)
                aplicar_material(app, design, f, 'PLA', avisos)

        # ---------- Resumen ----------
        recorrido = z_tuerca - (z_pol_hus + 16.0)
        z_tool = z_carro - c['vastago_largo'] - c['gripper_z']

        msg = [
            'Estacion 1 generada, {} componentes.'.format(raiz.occurrences.count),
            '',
            'Comprobacion contra las restricciones de la guia:',
            '  Alcance XY     {:.0f} mm   exigido 320 +/- 20'.format(c['l1'] + c['l2']),
            '  Altura total   {:.0f} mm   exigido 300 +/- 20'.format(z_tope),
            '  Efector        z de {:.0f} a {:.0f} mm'.format(
                z_tool - c['carrera_d3'], z_tool),
            '  Bandeja a 50 y traspaso a 60, ambos dentro del recorrido',
            '',
            'Recorrido del carro {:.0f} mm, carrera de diseno {:.0f} mm'.format(
                recorrido, c['carrera_d3']),
            '',
            'Siguiente paso: las uniones. El orden esta en la guia.',
        ]
        if avisos:
            msg += ['', 'Avisos:'] + ['  ' + a for a in avisos]

        ui.messageBox('\n'.join(msg), 'Proyecto R - Estacion 1')

    except Exception:
        if ui:
            ui.messageBox('Fallo el script:\n{}'.format(traceback.format_exc()),
                          'Proyecto R - Estacion 1')
