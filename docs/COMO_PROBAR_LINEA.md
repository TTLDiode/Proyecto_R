# Cómo probar la línea completa

Rama `prueba-linea`: las tres estaciones (E1, E2 y E3) con su control real y
el orquestador, en un mismo Gazebo. Respecto a `main` cambian ajustes de E1,
E3 y de la línea para que el ciclo cierre; la lista está en
[`docs/estacion2/cambios-linea-e1-e3.md`](estacion2/cambios-linea-e1-e3.md).

## Requisitos

- Ubuntu 24.04, ROS 2 Jazzy y Gazebo Harmonic.
- Paquetes que piden los `package.xml`: `ros-jazzy-ros-gz` (`ros_gz_sim`,
  `ros_gz_bridge`), `ros-jazzy-gz-ros2-control`, `ros2_control` y
  `ros2_controllers` (`joint_state_broadcaster`,
  `joint_trajectory_controller`), MoveIt 2 (`moveit_ros_move_group`,
  `moveit_planners_ompl`, `moveit_kinematics`, `moveit_configs_utils`…),
  `xacro`, `robot_state_publisher`, `rviz2`, `python3-numpy` y
  `python3-yaml`.
- Opcional: `terminator`, para que el script abra las 4 terminales en una
  sola ventana. Sin él usa `gnome-terminal`.

## Instalación y compilación

```bash
git clone https://github.com/TTLDiode/Proyecto_R.git   # o, si ya lo tienes: git fetch origin
cd Proyecto_R
git checkout prueba-linea
git pull

sudo rosdep init      # solo la primera vez en ese PC
rosdep update
rosdep install --from-paths src --ignore-src -y

source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Correr la línea

Con el script (desde la raíz del repo):

```bash
scripts/correr_linea.sh 3        # 3 piezas; sin número son 30
MODO=fondo scripts/correr_linea.sh 3   # todo en esta terminal, logs en ~/linea_logs
```

Abre E1 + Gazebo, luego la E2 a los 15 s, la E3 a los 25 s y el orquestador
a los 35 s.

A mano, en 4 terminales y en este orden, con `source install/setup.bash` en
cada una y unos segundos entre una y otra:

```bash
ros2 launch station1_description simulacion.launch.py
ros2 launch station2_description simulacion_en_linea.launch.py
ros2 launch station3_description simulacion_en_linea.launch.py
ros2 launch line_bringup line.launch.py mock_stations:=' ' n_parts:=3
```

## Qué se debe ver

- La E1 toma cada pieza de la bandeja y la deja en el traspaso 1-2; la E2 la
  inspecciona con la cámara, la clasifica por color y la deja en el traspaso
  2-3; la E3 la lleva a su caja: rojo → A, verde → B, azul → C.
- En la terminal del orquestador, una línea por pieza
  (`pieza N completada en … s, categoria …`) y al final
  `corrida terminada: … tiempo medio de linea …`. En la última corrida de
  6 piezas el tiempo medio fue de 108.9 s por pieza.
- Con las tres estaciones el factor de tiempo real (RTF) de Gazebo queda
  por debajo de 1, así que los tiempos dependen del PC.

## Solo la Estación 2

```bash
ros2 launch station2_description simulacion.launch.py
ros2 run station2_control vision
ros2 run station2_control nodo_estacion
ros2 run station2_control prueba_estacion
```

Imprime una tabla con las tres piezas clasificadas (A, B, C) y su error de
entrega (límite 20 mm), y la guarda en `resultados/estacion2/prueba_estacion.md`.

## Problemas comunes

- **Gazebo se cierra o la E1 no se mueve**: queda una simulación vieja
  cerrándose. El script ya la cierra; a mano:
  `pkill -f "gz sim"; pkill -f "ros2 launch"`, espera unos segundos y vuelve
  a lanzar.
- **El script no abre ventanas** (errores de `DISPLAY` o de DBUS, por ejemplo
  por SSH): corre `MODO=fondo scripts/correr_linea.sh 3`.
- **`No encuentro …/install/setup.bash`**: falta `colcon build` en la raíz
  del repo.
- **Trayectorias abortadas o el ciclo muy lento**: RTF bajo. Cierra otros
  programas, usa pocas piezas (`3`) o lanza la E1 con `gui:=false`.
