# Estación 2 · prueba del URDF (Entrega 1)

Probado el 24/10/2026 en Ubuntu 24.04 con ROS 2 Jazzy y Gazebo Harmonic, rama
`estacion2-ros` (216c8a1), compilando con
`colcon build --packages-up-to station2_description`.

| Archivo | Qué es |
|---|---|
| `check_urdf.txt` | `check_urdf` del URDF que genera `xacro` con `prefijo:=station2/ control:=false acople:=false camara:=false`. Lo analiza sin errores: raíz `world`, 10 frames. Sale igual que `../check_urdf.txt` |
| `rviz_display.png` | `ros2 launch station2_description display.launch.py`: el modelo en RViz2 con las articulaciones en cero |
| `arbol_tf.pdf` / `.png` | `ros2 run tf2_tools view_frames` con el display abierto: world → base_link → link_1 → link_2 → {link_3 → dedo_1, dedo_2, tool0; camara_link → camara_optica} |
| `gazebo_spawn.png` | `ros2 launch station2_description simulacion.launch.py`: el robot aparece en line.sdf con θ1 = 90° (posición inicial), las tres piezas de prueba y los soportes. Se cargan y activan joint_state_broadcaster, brazo_controller y mordaza_controller. No se mandó ninguna trayectoria |

En Gazebo, el log muestra un aviso de SDF sin consecuencias, sobre
`gz_frame_id` en el sensor de la cámara.

La captura de RViz2 se hizo con `xwd` sobre la ventana de render, y la de
Gazebo con el plugin Screenshot de un segundo cliente `gz sim -g` (con Wayland,
`xwd` sobre la ventana de Gazebo sale con rayas).
