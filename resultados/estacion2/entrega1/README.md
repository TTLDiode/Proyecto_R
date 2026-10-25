# Estación 2 · prueba del URDF (Entrega 1)

Probado el 24/10/2026 en Ubuntu 24.04, con ROS 2 Jazzy y Gazebo Harmonic, en
la rama `estacion2-ros`. Compilación: `colcon build --packages-up-to
station2_description station2_control`.

El visual del modelo son las mallas del CAD v5.7 con los colores y
materiales de cada pieza (`station2_description/meshes/`, generadas desde el
STEP del ensamble; ver `station2_description/scripts/README.md`):

- acrílico claro transparente;
- PLA blanco;
- motores Pololu;
- separadores de latón;
- tornillería negra y galvanizada;
- correas negras;
- PCB con sus componentes;
- grabado láser de la base: banderas, «TRÍADA», «SCARA RRP · Estación 2 ·
  Proyecto R» y el autor.

La colisión sigue siendo de cajas y cilindros.

| Archivo | Qué es |
|---|---|
| `check_urdf.txt` | `check_urdf` sobre el URDF que genera `xacro` (`prefijo:=station2/ control:=false acople:=false camara:=false`). Sin errores: raíz `world` y 16 frames, que son los 10 de la cadena más 6 `*_acrilico`, unidos con juntas fijas y solo visuales |
| `rviz_display.png` | `ros2 launch station2_description display.launch.py`: base en el origen, centrada en la rejilla, y brazo en la pose de reposo (θ1 = 90°, hacia +y). Vista desde el frente (−y, lado del grabado y del motor de θ1), con la PCB (+x) a la derecha |
| `arbol_tf.pdf` / `.png` | `ros2 run tf2_tools view_frames` con el modelo abierto. El árbol es world → base_link → link_1 → link_2, y de link_2 cuelgan link_3 (dedo_1, dedo_2, tool0) y camara_link → camara_optica. Cada eslabón lleva además su `<eslabon>_acrilico` fijo. Con θ1 = 90°, tool0 está en (0, 0.300, 0.120) |
| `gazebo_spawn.png` | `ros2 launch station2_description simulacion.launch.py`: la estación en su sitio de la línea, world (0.5, 0, 0) según `line_poses.yaml`, en reposo, con las tres piezas de prueba y los soportes. Se activan joint_state_broadcaster, brazo_controller y mordaza_controller |
| `prueba_estacion.md` | `ros2 run station2_control prueba_estacion` con el modelo de colores y la bajada de agarre corregida: 3 de 3 piezas bien clasificadas, peor error en planta de 2.3 mm (límite: 20 mm) y ciclos de unos 27 s |
| `gazebo_agarre.png` | Primer plano en Gazebo al coger la pieza en el traspaso 1-2: el cubo va entre los dedos, con las gomas en sus caras |
| `rviz_primeros_planos.png` | Primeros planos en RViz2 de la PCB, la mordaza con el servo, la torre de θ1 y el codo |
| `hoja_contacto/` | Las 381 piezas distintas del STEP, teseladas, para revisarlas una a una |

## Correcciones del 25/10/2026

- **Piezas con puntas**: la simplificación de triángulos deformaba las piezas
  curvas (husillo, condensadores, borneras, bobina, poleas, tornillos, PLA). Se
  quitó. Las mallas salen del teselado del STEP sin simplificar, y se
  comprobaron los 994 cuerpos contra su sólido: el área coincide con un error
  máximo del 0,8 % (`scripts/verificar_mallas.py`). La hoja de contacto se
  revisó a ojo. Para aligerar, solo se quitan las vías, el cobre de abajo y las
  paredes de 35 µm de las capas de las placas, que no se ven. Quedan 560 000
  triángulos y unos 31 MB; Gazebo va a tiempo real (RTF ≈ 1.0).
- **La pieza viajaba por debajo de los dedos**: el nodo bajaba tool0 (las
  puntas de los dedos) hasta z = 0.060, la altura de los traspasos, pero la
  pieza de 12 mm está apoyada en el soporte de z 0.042 a 0.054. El acople de
  Gazebo la fijaba 6 mm por debajo de los dedos. Ahora, al coger y al soltar,
  las puntas bajan 16 mm más, hasta 2 mm sobre el soporte (`AGARRE` en
  `nodo_estacion.py`, d3 = 76 mm), y las gomas de los dedos (12 mm de alto)
  quedan sobre las caras del cubo. Los puntos de la línea y el DH no cambian.
- **RViz**: el TF se muestra sin nombres ni flechas y con ejes pequeños, para
  que no tape el robot.

## Correcciones del 24/10/2026

- **Colores, materiales y grabado**: las mallas STL eran de un solo color. Ahora
  salen del STEP con el color de cada pieza, en Collada. El acrílico va aparte,
  en STL y con el color transparente del URDF, en un eslabón solo visual
  (RViz no lee la opacidad de las mallas Collada y aplica una sola opacidad
  por eslabón). El grabado de la base está en la geometría del STEP, a
  0,3 mm bajo la cara de arriba, y se pinta en oscuro.

- **Faltaban piezas**: el visual eran sólidos simples. Ahora lleva las mallas
  del CAD v5.7, exportadas de Fusion en la pose cero, en metros y en el marco
  de cada eslabón. El exportador había dejado las normales a cero y las piezas
  salían negras en RViz; se recalcularon con el orden de los vértices (el
  volumen con signo sale positivo, las caras miran hacia fuera). Con
  `mallas:=false` se vuelve a los sólidos simples.
- **Orientación de la base**: en los sólidos simples, la PCB estaba en +y y el
  motor de θ1 en −x, girados 90° respecto al CAD. Por eso, en reposo, el brazo
  quedaba encima de la PCB. Como en el CAD, la PCB va ahora en +x y el motor
  de θ1 en −y. No cambian ni la cinemática, ni los límites, ni los puntos de
  tarea.
- **Centrado en RViz**: `display.launch.py` pone la base en el origen
  (`base_x:=0`) y arranca en reposo. En Gazebo la estación sigue en x = 0.5,
  su sitio en la línea.
- **Rutas de las mallas**: van en `file://` con la ruta que resuelve xacro, de
  modo que Gazebo las encuentra también al añadir la estación al Gazebo de la
  Estación 1.

Capturas: la de RViz2 con `xwd` sobre la ventana de render, y la de Gazebo con
el plugin Screenshot de un segundo cliente `gz sim -g`. Las pruebas se
hicieron con `ROS_DOMAIN_ID=42` y `GZ_PARTITION` propia, para no mezclarse con
otra simulación abierta en la máquina.
