# Cambios de la rama `prueba-linea` respecto a `main`

Lo que hubo que tocar fuera de la Estación 2 para que la línea completa (E1,
E2 y E3 reales y el orquestador, en un mismo Gazebo) corra de punta a punta.
Cada punto dice el archivo y el commit donde está.

## Estación 1

- `station1_description/config/station1_controllers.yaml`: tolerancia de
  seguimiento de `joint_1` y `joint_2` de 0.5 a 0.8 rad. Con las tres
  estaciones en el mismo Gazebo el RTF baja y el brazo se retrasa más
  (ae61109).

## Estación 3

- `station3_description/launch/simulacion_en_linea.launch.py`:
  - usa la pieza `pieza`, la que deja la E2 en el traspaso 2-3, en lugar de
    `pieza_e3`, que solo sirve con la E2 simulada (ae61109);
  - ancla la base al world de Gazebo (`anclar_mundo:=true`); antes estaba
    suelta y giraba (40f620b);
  - la estación nace en su pose de espera, sobre la caja B, y no encima de
    `handoff_2_3`, donde la E2 chocaba con ella al dejar la pieza (b253859).
- `station3_description/urdf/station3.ros2_control.xacro`: argumentos
  `q1_inicial`, `q2_inicial` y `q3_inicial` para esa pose inicial (0 por
  defecto, como antes) (b253859).
- `station3_description/urdf/station3.urdf.xacro`: el link fijo `world` para
  el anclaje y la ventosa con `collide_bitmask` 0x02, para que no choque con
  la pieza ni la empuje (40f620b, 882bf60, 3e94d17).
- `station3_control/nodo_estacion.py`: la pose de espera (`PUNTO_ESPERA`) y
  la altura de agarre en el traspaso 2-3: la pieza llega sobre el soporte de
  42 mm de la E2, no en el suelo (`DZ_AGARRE_TRASPASO`) (ae61109, b253859,
  882bf60).

## Línea (orquestador y mundo)

- `line_orchestrator/reponedor.py`: la pieza nueva nace en la bandeja de la
  E1 (antes nacía en el origen, dentro de la base de la E1), se sueltan los
  acoples que nacen enganchados, rota los colores verde, rojo y azul, deja una
  copia fija de cada pieza en su caja y no repone tras la última pieza
  (3fc9272, b253859, d00838d).
- `line_bringup/launch/line.launch.py`: pasa `n_parts` al reponedor
  (d00838d).
- `line_bringup/worlds/line.sdf`: la pieza con `collide_bitmask` 0x01, para
  que no choque con la ventosa de la E3 (882bf60).

## Estación 2 (para referencia)

- `station2_description/config/station2_controllers.yaml`: tolerancia de
  seguimiento de `joint_1` y `joint_2` a 0.5 rad (ae61109).
- `station2_description/launch/simulacion.launch.py`: la altura de los
  soportes pasa a ser un parámetro (42 mm por defecto); el traspaso 2-3 queda
  con el mismo soporte de 42 mm que el puesto de inspección (a3307ba,
  882bf60). El código del nodo de la E2 queda igual que en `main`.
