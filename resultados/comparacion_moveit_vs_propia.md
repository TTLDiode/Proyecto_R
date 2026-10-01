# Comparacion MoveIt contra codigo propio

Estacion 1, 2026-09-29 05:41

| punto | q propia (deg, deg, mm) | q MoveIt | max |dq| | err IK propia | err IK MoveIt | err final propia | err final MoveIt |
|---|---|---|---|---|---|---|---|
| bandeja | 109.07, 100.98, 70.0 | 109.07, 100.98, 70.0 | 2.52e-05 | 0.000 mm | 0.004 mm | 0.000 mm | 1.770 mm |
| esquina1 | 131.33, 86.13, 70.0 | 131.33, 86.13, 70.0 | 6.96e-08 | 0.000 mm | 0.000 mm | 0.000 mm | 0.973 mm |
| esquina2 | 120.04, 62.34, 70.0 | 120.04, 62.34, 70.0 | 1.17e-08 | 0.000 mm | 0.000 mm | 0.000 mm | 0.828 mm |
| esquina3 | 108.92, 130.30, 70.0 | 108.92, 130.30, 70.0 | 6.43e-06 | 0.000 mm | 0.001 mm | 0.000 mm | 1.052 mm |
| esquina4 | 87.84, 104.48, 70.0 | 87.84, 104.48, 70.0 | 1.81e-08 | 0.000 mm | 0.000 mm | 0.000 mm | 1.437 mm |
| traspaso | 33.21, -77.98, 60.0 | 33.21, -77.98, 60.0 | 8.07e-07 | 0.000 mm | 0.000 mm | 0.001 mm | 0.816 mm |

| punto | t IK propia | t IK MoveIt | t plan MoveIt | t ejecucion propia | t ejecucion MoveIt |
|---|---|---|---|---|---|
| bandeja | 68.5 us | 0.006 s | 0.064 s | 8.858 s | 4.866 s |
| esquina1 | 19.4 us | 0.006 s | 0.042 s | 8.955 s | 4.927 s |
| esquina2 | 15.3 us | 0.009 s | 0.073 s | 8.909 s | 5.262 s |
| esquina3 | 14.1 us | 0.003 s | 0.062 s | 9.655 s | 5.130 s |
| esquina4 | 22.0 us | 0.005 s | 0.062 s | 10.156 s | 5.569 s |
| traspaso | 17.2 us | 0.040 s | 0.079 s | 8.105 s | 4.930 s |

Par maximo que exige cada trayectoria, evaluado con el mismo modelo dinamico. El presupuesto es 0.389 N.m, el par que deja pasar el TB6612FNG.

| punto | duracion propia | par propia | duracion MoveIt | par MoveIt | par MoveIt / presupuesto |
|---|---|---|---|---|---|
| bandeja | 7.02 s | 0.002 N.m | 3.80 s | 0.085 N.m | 22 % |
| esquina1 | 7.02 s | 0.004 N.m | 3.82 s | 0.127 N.m | 33 % |
| esquina2 | 7.02 s | 0.003 N.m | 3.81 s | 0.110 N.m | 28 % |
| esquina3 | 7.02 s | 0.003 N.m | 3.80 s | 0.113 N.m | 29 % |
| esquina4 | 7.02 s | 0.000 N.m | 3.80 s | 0.013 N.m | 3 % |
| traspaso | 6.02 s | 0.031 N.m | 3.34 s | 0.268 N.m | 69 % |
