# -*- coding: utf-8 -*-
"""
Estacion 1 - Alimentacion y singulacion. Proyecto R, Linea Simulada.

Construye el esqueleto de la estacion en Fusion con la misma geometria que el
modelo URDF del repositorio, de modo que ambos describan el mismo mecanismo.
Las cotas provienen de:

    src/station1_description/urdf/parametros.xacro

Arquitectura de transmision por correa. Los motores Pololu 37D miden 37 mm de
diametro y pesan del orden de 200 g, de modo que montarlos sobre las
articulaciones rompia la restriccion de altura y dejaba la mayor parte de la
masa en voladizo:

    theta1  motor alojado dentro de la columna, accionamiento directo
    theta2  motor sobre el eslabon 1 y coaxial con theta1, correa al codo
    d3      motor vertical junto al codo, correa al husillo del extremo

El eje vertical es un husillo fijo al eslabon 2 con la tuerca desplazandose
sobre el, y el efector cuelga de un vastago guiado. La altura maxima de la
estacion la fija el extremo del husillo, que no se mueve.

Que hace:
  - Crea los parametros de usuario con los valores del URDF.
  - Genera la estructura y las envolventes de los actuadores en su posicion.
  - Importa los modelos CAD reales del fabricante y los deja apartados, listos
    para posicionar: la orientacion interna de cada STEP es desconocida, de
    modo que emparejarlos es trabajo manual.

Que NO hace, y queda como trabajo de diseno:
  - Las uniones y los emparejamientos de los componentes importados.
  - Poleas, correa, tensor y rodamientos.
  - Soportes de motor, taladros, chaflanes y tolerancias.

Uso: Utilidades > ADD-INS > Scripts, seleccionar EstacionR1 y Ejecutar.
"""
import os
import traceback

import adsk.core
import adsk.fusion

# --------------------------------------------------------------------------
# Cotas en milimetros, identicas a parametros.xacro
# --------------------------------------------------------------------------
COTAS = {
    'l1':              180.0,
    'l2':              140.0,
    'altura_columna':  170.0,
    'husillo_largo':   145.0,
    'husillo_radio':     4.0,
    'carrera_d3':      120.0,
    'vastago_largo':   150.0,
    'vastago_radio':     8.0,
    'tuerca_lado':      30.0,
    'base_x':          220.0,
    'base_y':          220.0,
    'base_z':           12.0,
    'columna_radio':    30.0,
    'motor2_offset':    60.0,
    'motor3_offset':    30.0,
    'brazo_ancho':      50.0,
    'brazo_espesor':     8.0,
    'motor_diametro':   37.0,
    'motor_largo':      95.0,
    'motor_eje_diam':    6.0,
    'motor_brida_diam': 16.0,
    'motor_tornillo':    3.0,
    'gripper_x':        40.0,
    'gripper_y':        30.0,
    'gripper_z':        35.0,
}

MATERIALES = {
    'MDF':      ['MDF', 'Medium Density Fiberboard', 'Madera', 'Wood'],
    'ACRILICO': ['Acrylic', 'ABS Plastic', 'Plastic', 'Acrilico'],
    'PLA':      ['PLA', 'ABS Plastic', 'Plastic'],
    'ACERO':    ['Steel', 'Acero', 'Stainless Steel'],
    'ALUMINIO': ['Aluminum', 'Aluminio', 'Aluminum 6061'],
}

# Modelos del fabricante. Se importan y se dejan apartados del mecanismo.
CARPETA_CAD = os.path.expanduser(
    '~/Documents/proyecto-r-linea-simulada/cad')

# Interfaz mecanica del motor, medida sobre Gearmotor_37D_100.stp:
#   cuerpo 36.8 mm de diametro, 94.6 mm de largo total
#   eje de salida 6 mm, brida de centraje 16 mm, fijacion M3
# El STEP del servo SG90 es un ensamble con las piezas separadas mas de un
# metro entre si, de modo que no se importa: se usa su envolvente.
COMPONENTES = [
    ('Motor Pololu 37D con reductora',
     'Gearmotor_37D_100.stp'),
    ('Raspberry Pi Pico',
     'raspberry pi pico/raspberry pi pico.step'),
    ('Fin de carrera KW3-OZ',
     'kw3-oz-switch-fin-de-carrera-1.snapshot.4/KW3-OZ.STEP'),
    ('Driver TB6612FNG',
     'doble-puente-h-tb6612fng-1.snapshot.1/DOBLE PUENTE H.step'),
]

IMPORTAR_COMPONENTES = True   # ponlo en False si la importacion tarda demasiado


def mm(valor):
    """Fusion trabaja internamente en centimetros."""
    return valor / 10.0


def crear_parametros(design):
    params = design.userParameters
    creados = 0
    for nombre, valor in sorted(COTAS.items()):
        if params.itemByName(nombre):
            continue
        params.add(nombre,
                   adsk.core.ValueInput.createByString('{} mm'.format(valor)),
                   'mm', 'Cota del modelo URDF de la Estacion 1')
        creados += 1
    return creados


def buscar_material(app, candidatos):
    for libreria in app.materialLibraries:
        for nombre in candidatos:
            material = libreria.materials.itemByName(nombre)
            if material:
                return material
    return None


def aplicar_material(app, design, componente, clave, avisos):
    material = buscar_material(app, MATERIALES[clave])
    if not material:
        avisos.append('No se encontro material para {}'.format(clave))
        return
    for cuerpo in componente.bRepBodies:
        try:
            cuerpo.material = material
        except Exception:
            try:
                local = design.materials.itemByName(material.name)
                if not local:
                    local = design.materials.addByCopy(material, material.name)
                cuerpo.material = local
            except Exception:
                avisos.append('No se pudo asignar {} a {}'.format(
                    clave, componente.name))


def nuevo_componente(raiz, nombre):
    occ = raiz.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = nombre
    return occ.component


def extruir(componente, perfil, z_inicio, altura):
    extrusiones = componente.features.extrudeFeatures
    entrada = extrusiones.createInput(
        perfil, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    entrada.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(altura)))
    if abs(z_inicio) > 1e-9:
        entrada.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(mm(z_inicio)))
    return extrusiones.add(entrada)


def caja(raiz, nombre, x0, y0, z0, dx, dy, dz):
    comp = nuevo_componente(raiz, nombre)
    croquis = comp.sketches.add(comp.xYConstructionPlane)
    croquis.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(mm(x0), mm(y0), 0),
        adsk.core.Point3D.create(mm(x0 + dx), mm(y0 + dy), 0))
    extruir(comp, croquis.profiles.item(0), z0, dz)
    return comp


def cilindro_z(raiz, nombre, x0, y0, z0, radio, altura):
    comp = nuevo_componente(raiz, nombre)
    croquis = comp.sketches.add(comp.xYConstructionPlane)
    croquis.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(mm(x0), mm(y0), 0), mm(radio))
    extruir(comp, croquis.profiles.item(0), z0, altura)
    return comp


def importar_componentes(app, raiz, avisos):
    """Importa los STEP del fabricante y los aparta del mecanismo.

    La orientacion interna de cada archivo es desconocida, de modo que se
    colocan en fila delante de la maquina para emparejarlos despues.
    """
    gestor = app.importManager
    importados = 0
    y_fila = -260.0
    x_fila = -200.0
    for etiqueta, relativa in COMPONENTES:
        ruta = os.path.join(CARPETA_CAD, relativa)
        if not os.path.exists(ruta):
            avisos.append('No se encontro {}'.format(relativa))
            continue
        try:
            antes = raiz.occurrences.count
            opciones = gestor.createSTEPImportOptions(ruta)
            opciones.isViewFit = False
            gestor.importToTarget(opciones, raiz)
            for i in range(antes, raiz.occurrences.count):
                occ = raiz.occurrences.item(i)
                occ.component.name = etiqueta
                matriz = occ.transform
                matriz.translation = adsk.core.Vector3D.create(
                    mm(x_fila), mm(y_fila), 0)
                occ.transform = matriz
            x_fila += 120.0
            importados += 1
        except Exception as e:
            avisos.append('Fallo al importar {}: {}'.format(etiqueta, e))
    return importados


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        raiz = design.rootComponent

        n_params = crear_parametros(design)
        avisos = []
        c = COTAS

        # Alturas derivadas, calculadas igual que en el URDF.
        # Cara inferior del eslabon 1: la columna termina aqui, no lo atraviesa.
        z_link1 = c['altura_columna'] - c['brazo_espesor'] / 2.0
        z_link2 = c['altura_columna'] - c['brazo_espesor'] * 1.5
        z_base_husillo = c['altura_columna'] - c['brazo_espesor']
        z_top_husillo = z_base_husillo + c['husillo_largo']
        x_codo = c['l1']
        x_husillo = c['l1'] + c['l2']

        # ---------------- Base ----------------
        base = caja(raiz, 'base_link',
                    -c['base_x'] / 2.0, -c['base_y'] / 2.0, 0.0,
                    c['base_x'], c['base_y'], c['base_z'])
        aplicar_material(app, design, base, 'MDF', avisos)

        columna = cilindro_z(raiz, 'columna', 0.0, 0.0, c['base_z'],
                             c['columna_radio'],
                             z_link1 - c['base_z'])
        aplicar_material(app, design, columna, 'PLA', avisos)

        # Motor de theta1, alojado dentro de la columna.
        m1 = cilindro_z(raiz, 'motor_theta1', 0.0, 0.0,
                        z_link1 - c['motor_largo'],
                        c['motor_diametro'] / 2.0, c['motor_largo'])
        aplicar_material(app, design, m1, 'ALUMINIO', avisos)

        # ---------------- Brazo ----------------
        link1 = caja(raiz, 'link_1', 0.0, -c['brazo_ancho'] / 2.0,
                     z_link1,
                     c['l1'], c['brazo_ancho'], c['brazo_espesor'])
        aplicar_material(app, design, link1, 'ACRILICO', avisos)

        # Motor de theta2, sobre el eslabon 1 y coaxial con el eje de theta1.
        # Asi la distancia a la polea del codo no cambia al girar el brazo, y
        # su masa queda a radio cero del eje.
        m2 = cilindro_z(raiz, 'motor_theta2', 0.0, 0.0,
                        z_link1 + c['brazo_espesor'],
                        c['motor_diametro'] / 2.0, c['motor_largo'])
        aplicar_material(app, design, m2, 'ALUMINIO', avisos)

        link2 = caja(raiz, 'link_2', x_codo, -c['brazo_ancho'] / 2.0,
                     z_link2 - c['brazo_espesor'] / 2.0,
                     c['l2'], c['brazo_ancho'], c['brazo_espesor'])
        aplicar_material(app, design, link2, 'ACRILICO', avisos)

        # Motor de d3, vertical sobre el eslabon 2 y paralelo al husillo:
        # una correa dentada exige ejes paralelos.
        m3 = cilindro_z(raiz, 'motor_d3', x_codo + c['motor3_offset'], 0.0,
                        z_link2 + c['brazo_espesor'] / 2.0,
                        c['motor_diametro'] / 2.0, c['motor_largo'])
        aplicar_material(app, design, m3, 'ALUMINIO', avisos)

        # ---------------- Eje vertical ----------------
        husillo = cilindro_z(raiz, 'husillo', x_husillo, 0.0, z_base_husillo,
                             c['husillo_radio'], c['husillo_largo'])
        aplicar_material(app, design, husillo, 'ACERO', avisos)

        tuerca = caja(raiz, 'tuerca', x_husillo - c['tuerca_lado'] / 2.0,
                      -c['tuerca_lado'] / 2.0,
                      z_top_husillo - c['tuerca_lado'] / 4.0,
                      c['tuerca_lado'], c['tuerca_lado'], c['tuerca_lado'] / 2.0)
        aplicar_material(app, design, tuerca, 'ACERO', avisos)

        vastago = cilindro_z(raiz, 'vastago', x_husillo, 0.0,
                             z_top_husillo - c['vastago_largo'],
                             c['vastago_radio'], c['vastago_largo'])
        aplicar_material(app, design, vastago, 'ALUMINIO', avisos)

        z_gripper = z_top_husillo - c['vastago_largo']
        gripper = caja(raiz, 'gripper_link',
                       x_husillo - c['gripper_x'] / 2.0,
                       -c['gripper_y'] / 2.0,
                       z_gripper - c['gripper_z'],
                       c['gripper_x'], c['gripper_y'], c['gripper_z'])
        aplicar_material(app, design, gripper, 'PLA', avisos)

        n_importados = 0
        if IMPORTAR_COMPONENTES:
            n_importados = importar_componentes(app, raiz, avisos)

        alcance = c['l1'] + c['l2']
        z_tool = z_gripper - c['gripper_z']

        mensaje = [
            'Estacion 1 generada.',
            '',
            'Parametros de usuario: {}'.format(n_params),
            'Componentes del fabricante importados: {}'.format(n_importados),
            '',
            'Comprobacion contra las restricciones de la guia:',
            '  Alcance XY:   {:.0f} mm   (exigido 320 +/- 20)'.format(alcance),
            '  Altura total: {:.0f} mm   (exigido 300 +/- 20)'.format(z_top_husillo),
            '  Efector:      z de {:.0f} a {:.0f} mm, tarea a 50 y 60'.format(
                z_tool - c['carrera_d3'], z_tool),
            '',
            'Siguiente paso:',
            '  1. Emparejar los componentes importados, que estan apartados',
            '     delante de la maquina, con sus envolventes.',
            '  2. Anadir las uniones: revoluta en la base, revoluta en el codo',
            '     en x = {:.0f} mm, y deslizante del vastago con {:.0f} mm.'.format(
                x_codo, c['carrera_d3']),
            '  3. Disenar poleas, correa y soportes de motor.',
        ]
        if avisos:
            mensaje += ['', 'Avisos:'] + ['  ' + a for a in avisos]

        ui.messageBox('\n'.join(mensaje), 'Proyecto R - Estacion 1')

    except Exception:
        if ui:
            ui.messageBox('Fallo el script:\n{}'.format(traceback.format_exc()),
                          'Proyecto R - Estacion 1')
