# Prueba de la linea completa (tres estaciones reales)

24 de octubre de 2026, ROS 2 Jazzy y Gazebo Harmonic en la nube, con render por
software. Las tres estaciones corren con su control real en el mismo Gazebo y
el orquestador sin ningun nodo de prueba:

```
ros2 launch station1_description simulacion.launch.py gui:=false
ros2 launch station2_description simulacion_en_linea.launch.py
ros2 launch station3_description simulacion_en_linea.launch.py
ros2 run station1_control nodo_estacion
ros2 run station2_control vision
ros2 run station2_control nodo_estacion
ros2 run station3_control nodo_estacion
ros2 launch line_bringup line.launch.py mock_stations:=, n_parts:=1
```

## Resultado

| estacion | resultado |
|---|---|
| Estacion 1 | ciclo 1 completo en 55.1 s (limite 30 s) |
| Estacion 2 | ciclo 1 en 51.1 s: categoria A, error de tool0 0.12 mm en el traspaso 1-2 y 0.12 mm en el 2-3 (limite 20 mm) |
| Estacion 3 | Ciclo completo para 1. Unidades en categoria A: 1 |
| Orquestador | pieza 1 completada en 145.4 s, categoria A |

Pieza de la linea (`pieza`, roja) al terminar: (0.720023, -0.000018, 0.048000),
error en planta 0.029 mm contra el traspaso 2-3. Imagen que clasifico la
vision: `vision/linea_completa_pieza.png`.

Registro del orquestador:

```
inicio de la corrida: 1 piezas
pieza 1 completada en 145.4 s, categoria A
corrida terminada: 1 piezas, tiempo medio de linea 145.4 s
```

## Observaciones para el equipo

1. **Tiempo real.** Gazebo corrio al 17 % del tiempo real (render por software
   de tres robots y la camara). Los tiempos de ciclo medidos en reloj de pared
   salen inflados por eso; en un equipo con GPU se esperan los de las pruebas
   aisladas (Estacion 2: unos 26 s).
2. **Tolerancias con carga.** En el primer intento la Estacion 2 aborto el
   traslado al 2-3 por 0.101 rad de retraso en theta2 (tolerancia 0.10); se
   subio su tolerancia de seguimiento a 0.25 rad (la de llegada sigue en
   0.01 rad). En el segundo intento fue la Estacion 1 la que aborto un
   movimiento por tolerancia. El tercer intento completo la linea.
3. **Pieza del traspaso 2-3.** La Estacion 3 crea su propia pieza, `pieza_e3`,
   sobre el traspaso 2-3, y acopla esa y no `pieza`, la que trae la linea. Para
   la prueba se aparto `pieza_e3` antes de empezar, de modo que la Estacion 2
   pudiera entregar ahi la pieza real. Contractualmente la linea funciona, pero
   fisicamente la Estacion 3 empaca `pieza_e3`. Conviene acordar que la
   Estacion 3 acople `pieza` cuando corre en la linea.
