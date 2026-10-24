# Comparacion MoveIt contra codigo propio

Estacion 2 (TRIADA), 2026-10-24 09:14. Generado por `ros2 run station2_control comparar_moveit`.

| punto | objetivo world (m) | q propia (deg, deg, mm) | q MoveIt (KDL) | max dq | err IK propia (mm) | err IK MoveIt (mm) | err final propia (mm) | err final MoveIt (mm) |
|---|---|---|---|---|---|---|---|---|
| traspaso_12 | (0.250, 0.000, 0.060) | 148.90, 67.28, 60.0 (arriba) | 148.90, 67.29, 60.0 (arriba) | 2.78e-05 | 0.000 | 0.003 | 0.001 | 0.021 |
| inspeccion | (0.500, 0.200, 0.060) | 45.95, 96.67, 60.0 (arriba) | 45.95, 96.67, 60.0 (arriba) | 7.74e-07 | 0.000 | 0.000 | 0.001 | 0.095 |
| traspaso_23 | (0.720, 0.000, 0.060) | 39.40, -85.90, 60.0 (abajo) | 39.40, -85.90, 60.0 (abajo) | 6.18e-08 | 0.000 | 0.000 | 0.000 | 0.166 |
| libre_1 | (0.400, 0.250, 0.080) | 87.45, 52.46, 40.0 (arriba) | 136.15, -52.46, 40.0 (abajo) | 7.82e-06 | 0.000 | 0.001 | 0.000 | 0.016 |
| libre_2 | (0.700, 0.150, 0.100) | 5.77, 67.28, 20.0 (arriba) | 67.97, -67.28, 20.0 (abajo) | 2.08e-07 | 0.000 | 0.000 | 0.002 | 0.078 |

| punto | t IK propia | t IK MoveIt | t plan MoveIt | duracion perfil propio | duracion trayectoria MoveIt | t ejecucion propia | t ejecucion MoveIt |
|---|---|---|---|---|---|---|---|
| traspaso_12 | 55.6 us | 25.6 ms | 212.6 ms | 2.01 s | 1.35 s | 2.66 s | 1.68 s |
| inspeccion | 48.4 us | 9.1 ms | 51.6 ms | 2.01 s | 1.46 s | 2.60 s | 1.81 s |
| traspaso_23 | 40.3 us | 7.5 ms | 28.7 ms | 2.01 s | 1.42 s | 2.51 s | 1.76 s |
| libre_1 | 92.5 us | 4.2 ms | 28.5 ms | 1.34 s | 1.03 s | 1.70 s | 1.31 s |
| libre_2 | 36.4 us | 7.4 ms | 22.4 ms | 1.46 s | 1.08 s | 1.80 s | 1.48 s |
