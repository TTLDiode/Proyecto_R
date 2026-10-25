# Comparacion MoveIt contra codigo propio

Estacion 2 (TRIADA), 2026-10-25 01:58. Generado por `ros2 run station2_control comparar_moveit`.

| punto | objetivo world (m) | q propia (deg, deg, mm) | q MoveIt (KDL) | max dq | err IK propia (mm) | err IK MoveIt (mm) | err final propia (mm) | err final MoveIt (mm) |
|---|---|---|---|---|---|---|---|---|
| traspaso_12 | (0.250, 0.000, 0.060) | 148.90, 67.28, 60.0 (arriba) | 148.90, 67.28, 60.0 (arriba) | 1.64e-06 | 0.000 | 0.000 | 0.003 | 0.036 |
| inspeccion | (0.500, 0.200, 0.060) | 45.95, 96.67, 60.0 (arriba) | 134.05, -96.67, 60.0 (abajo) | 1.31e-07 | 0.000 | 0.000 | 0.000 | 0.050 |
| traspaso_23 | (0.720, 0.000, 0.060) | 39.40, -85.90, 60.0 (abajo) | 39.40, -85.90, 60.0 (abajo) | 3.10e-07 | 0.000 | 0.000 | 0.056 | 0.124 |
| libre_1 | (0.400, 0.250, 0.080) | 87.45, 52.46, 40.0 (arriba) | 87.45, 52.46, 40.0 (arriba) | 1.50e-06 | 0.000 | 0.000 | 0.005 | 0.455 |
| libre_2 | (0.700, 0.150, 0.100) | 5.77, 67.28, 20.0 (arriba) | 67.97, -67.28, 20.0 (abajo) | 2.08e-07 | 0.000 | 0.000 | 0.000 | 0.032 |

| punto | t IK propia | t IK MoveIt | t plan MoveIt | duracion perfil propio | duracion trayectoria MoveIt | t ejecucion propia | t ejecucion MoveIt |
|---|---|---|---|---|---|---|---|
| traspaso_12 | 52.9 us | 20.9 ms | 38.4 ms | 2.01 s | 1.35 s | 2.05 s | 1.41 s |
| inspeccion | 28.0 us | 2.4 ms | 29.3 ms | 2.01 s | 1.47 s | 2.05 s | 1.51 s |
| traspaso_23 | 51.5 us | 70.5 ms | 27.0 ms | 2.01 s | 1.42 s | 2.05 s | 1.46 s |
| libre_1 | 30.7 us | 2.1 ms | 22.5 ms | 1.34 s | 1.03 s | 1.35 s | 1.06 s |
| libre_2 | 71.0 us | 3.2 ms | 24.4 ms | 1.46 s | 1.08 s | 1.50 s | 1.11 s |
