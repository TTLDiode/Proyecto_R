# Estacion 1 y estacion 3 reales en un solo Gazebo

Estacion 2 simulada. La estacion 3 usa su propia pieza (`pieza_e3`).
Un proceso por terminal, en este orden.

1. Estacion 1 y Gazebo (con la ruta de recursos de la estacion 3):

       export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:<workspace>/install/station3_description/share
       ros2 launch station1_description simulacion.launch.py

   Sin el `export` el robot de la estacion 3 aparece sin mallas.

2. Estacion 3 en el mismo Gazebo:

       ros2 launch station3_description simulacion_en_linea.launch.py

3. Nodos de las estaciones:

       ros2 run station3_control nodo_estacion
       ros2 run station1_control nodo_estacion

4. Estacion 2 simulada:

       ros2 run line_orchestrator station_mock --ros-args -p station_id:=station2 -p duration_s:=3.0 -p assign_category:=true -p poses_file:=$(ros2 pkg prefix line_bringup)/share/line_bringup/config/line_poses.yaml

5. Orquestador, con una pieza:

       ros2 run line_orchestrator orchestrator --ros-args -p n_parts:=1 -p done_timeout_s:=150.0 -p poses_file:=$(ros2 pkg prefix line_bringup)/share/line_bringup/config/line_poses.yaml

   Con `done_timeout_s` por defecto (60 s) el orquestador aborta, porque los
   ciclos reales duran mas.

## Verificado
Una pieza, categoria B: `pieza_e3` quedo en el slot 1 de B (0.8099, -0.0399 en
world) y la pieza roja en el traspaso 1->2 (0.2500, 0.0000, 0.048).

## No probado
- `n_parts` mayor que 1: nadie repone las piezas entre vueltas.
- Categorias A y C a traves del orquestador.
- Una pieza fisica pasando de una estacion a otra (la estacion 2 es simulada).
