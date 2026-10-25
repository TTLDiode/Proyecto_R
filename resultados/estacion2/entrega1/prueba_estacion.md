# Prueba aislada de la Estacion 2

Generado por `ros2 run station2_control prueba_estacion`. Pieza colocada en el traspaso 1-2 (0.250, 0.000, 0.048); destino traspaso 2-3 (0.720, 0.000), centro de la pieza a z = 0.048.

| pieza | categoria esperada | categoria asignada | ciclo (s) | posicion final (m) | error en planta (mm) | error en altura (mm) |
|---|---|---|---|---|---|---|
| pieza_roja | A | A | 27.3 | (0.720000, 0.000000, 0.048000) | 0.000 | 0.000 |
| pieza_verde | B | B | 27.6 | (0.720000, 0.000000, 0.048000) | 0.000 | 0.000 |
| pieza_azul | C | C | 27.9 | (0.720000, 0.000000, 0.048000) | 0.000 | 0.000 |

Clasificacion correcta: 3 de 3. Peor error de posicionamiento en planta: 0.000 mm (limite de la guia: 20 mm).
