# Estación 2 · prueba del URDF (Entrega 1)

Probado el 24/10/2026 en Ubuntu 24.04, con ROS 2 Jazzy y Gazebo Harmonic, en
la rama `estacion2-ros`. Compilación: `colcon build --packages-up-to
station2_description station2_control`.

El modelo usa como visual las mallas STL del CAD v5.7 (`station2_description/
meshes/`). La colisión sigue siendo de cajas y cilindros.

| Archivo | Qué es |
|---|---|
| `check_urdf.txt` | `check_urdf` sobre el URDF que genera `xacro` (`prefijo:=station2/ control:=false acople:=false camara:=false base_x:=0.0`). Sin errores: raíz `world` y 10 frames |
| `rviz_display.png` | `ros2 launch station2_description display.launch.py`: base en el origen, centrada en la rejilla, y brazo en la pose de reposo (θ1 = 90°, hacia +y). Vista desde el frente (−y, lado del grabado y del motor de θ1), con la PCB (+x) a la derecha |
| `arbol_tf.pdf` / `.png` | `ros2 run tf2_tools view_frames` con el display abierto. El árbol es world → base_link → link_1 → link_2, y de link_2 cuelgan link_3 (dedo_1, dedo_2, tool0) y camara_link → camara_optica. Con θ1 = 90°, tool0 está en (0, 0.300, 0.120) |
| `gazebo_spawn.png` | `ros2 launch station2_description simulacion.launch.py`: la estación en su sitio de la línea, world (0.5, 0, 0) según `line_poses.yaml`, en reposo, con las tres piezas de prueba y los soportes. Se activan joint_state_broadcaster, brazo_controller y mordaza_controller |
| `prueba_estacion.md` | `ros2 run station2_control prueba_estacion` con el modelo de mallas: 3 de 3 piezas bien clasificadas y error en planta de 0.000 mm |

## Correcciones del 24/10/2026

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
