# Prueba de la linea con el orquestador

Estacion 2 real en Gazebo (control, vision y entrega), Estaciones 1 y 3 como nodos de prueba, `alimentador_linea` colocando las piezas de color en el traspaso 1-2. Comandos:

```
ros2 launch station2_description simulacion.launch.py gui:=false
ros2 run station2_control vision
ros2 run station2_control nodo_estacion
ros2 run station2_control alimentador_linea
ros2 launch line_bringup line.launch.py mock_stations:=station1,station3
```

Registro del orquestador y de la Estacion 2:

```
[INFO] [1792833362.822567731] [orchestrator]: inicio de la corrida: 3 piezas
[INFO] [1792833395.533962944] [station3_mock]: station3: pieza 1 entregada, categoria A
[INFO] [1792833395.538773857] [orchestrator]: pieza 1 completada en 32.7 s, categoria A
[INFO] [1792833427.454039705] [station3_mock]: station3: pieza 2 entregada, categoria B
[INFO] [1792833427.455860088] [orchestrator]: pieza 2 completada en 31.9 s, categoria B
[INFO] [1792833459.512386271] [station3_mock]: station3: pieza 3 entregada, categoria C
[INFO] [1792833459.515746229] [orchestrator]: pieza 3 completada en 32.1 s, categoria C
[INFO] [1792833459.516107557] [orchestrator]: corrida terminada: 3 piezas, tiempo medio de linea 32.2 s
[INFO] [1792833392.521266912] [station2_nodo_estacion]: ciclo 4 en 26.7 s: categoria A, error de tool0 0.00 mm en el traspaso 1-2 y 0.00 mm en el 2-3 (limite 20 mm)
[INFO] [1792833424.447751186] [station2_nodo_estacion]: ciclo 5 en 25.9 s: categoria B, error de tool0 0.00 mm en el traspaso 1-2 y 0.00 mm en el 2-3 (limite 20 mm)
[INFO] [1792833456.488529174] [station2_nodo_estacion]: ciclo 6 en 26.0 s: categoria C, error de tool0 0.00 mm en el traspaso 1-2 y 0.00 mm en el 2-3 (limite 20 mm)
```
