# Mallas con color de la Estación 2

Las mallas de `meshes/` salen del STEP del ensamble (CAD v5.7), que trae el
color de cada pieza:
`cad/station2/Estacion2_SCARA_step.zip`, en la rama `estacion2-cad`.

| Malla | Qué lleva |
|---|---|
| `<eslabon>.dae` | Las piezas opacas, en Collada, con un material por color |
| `<eslabon>_acrilico.stl` | Las piezas de acrílico; el URDF las pinta de acrílico claro transparente |

Los eslabones son base_link, link_1, link_2, link_3, dedo_1 y dedo_2. Todas
las mallas están en la pose cero, en metros y en el marco de su eslabón.

Para regenerarlas hace falta OpenCASCADE, que se instala en un entorno
virtual aparte, sin tocar ROS:

    python3 -m venv ~/venv_occ
    ~/venv_occ/bin/pip install cadquery-ocp numpy trimesh fast-simplification scipy networkx
    unzip Estacion2_SCARA_step.zip
    ~/venv_occ/bin/python step_a_mallas.py Estacion2_SCARA.step mallas.pkl 0.2   # ~1,5 min
    ~/venv_occ/bin/python mallas_dae.py mallas.pkl ../meshes

Qué hace `step_a_mallas.py`:

- Teselado del STEP con el color de cada cara.
- Agrupación por componente de primer nivel (base_fija, eslabon_1,
  eslabon_2, efector_d3, pinon, dedo_pos, dedo_neg).

Qué hace `mallas_dae.py`:

- Gira −90° en z todo lo que no es la base: el STEP está en reposo, con
  θ1 = 90°.
- Pasa cada grupo al marco de su eslabón, con los mismos orígenes que
  `cad/station2/urdf_cad/scripts/mallas_fusion.py`.
- Colores:
  - Acrílico claro: el STEP no trae la transparencia.
  - Tornillería galvanizada.
  - Grabado de la placa base en oscuro: son las caras a 0,3 mm bajo la cara
    de arriba.
- Simplifica las piezas pesadas para que Gazebo vaya fluido (unos 310 000
  triángulos en total).

El acrílico va en un eslabón aparte, solo visual, porque RViz no lee la
opacidad de las mallas Collada y aplica una sola opacidad a todo el eslabón.
