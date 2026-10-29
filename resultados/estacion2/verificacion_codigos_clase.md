# Entrega 2 · códigos de clase adaptados a la Estación 2

Scripts:

- `matlab/station2/e2_dk_TH_DH.m`: directa, adaptada de `ri_dk_ms3r_TH_DH.m`.
- `matlab/station2/e2_ik_geometrico_trayectoria.m`: inversa geométrica con
  trayectoria, adaptada de `ri_dk_ik_ms3r_trajectory.m`.

Del código de clase solo cambian la tabla DH y los puntos evaluados.

Tabla DH en la convención del código de clase (DH modificado,
`[a_ij alpha_ij s_j theta_j]`, mm):

| fila | a | α | s (d) | θ |
|---|---|---|---|---|
| {0}→{1} | 0 | 0 | 114 | θ1 |
| {1}→{2} | 160 | 0 | 6 | θ2 |
| {2}→{3} | 140 | π | d3 | 0 |
| {3}→{T} | 0 | 0 | 0 | 0 |

Es la misma cadena que la tabla DH estándar de `entrega2.md`.

En la máquina de Ubuntu no hay MATLAB. Las cuentas de los dos scripts se
reprodujeron en Python, operación por operación (25/10/2026), con los cinco
puntos de `comparacion_moveit_vs_propia.md`.

**Directa** (q de la tabla de MoveIt → posición en el mundo):

| punto | q (°, °, mm) | resultado (mm) | diferencia con el objetivo |
|---|---|---|---|
| traspaso_12 | 148.90, 67.28, 60 | (249.994, 0.000, 60.000) | 0.006 mm |
| inspeccion | 45.95, 96.67, 60 | (499.998, 199.991, 60.000) | 0.009 mm |
| traspaso_23 | 39.40, −85.90, 60 | (720.007, 0.004, 60.000) | 0.008 mm |
| libre_1 | 87.45, 52.46, 40 | (400.014, 250.000, 80.000) | 0.014 mm |
| libre_2 | 5.77, 67.28, 20 | (700.005, 150.004, 100.000) | 0.006 mm |

Las diferencias vienen solo de que los ángulos de entrada van redondeados a
dos decimales.

**Inversa geométrica frente a MoveIt (KDL)**, en la misma rama de codo:

| punto | propia (°, °, mm) | MoveIt | máx. diferencia |
|---|---|---|---|
| traspaso_12 | 148.90, 67.28, 60 | 148.90, 67.29, 60 | 0.006 |
| inspeccion | 45.95, 96.67, 60 | 45.95, 96.67, 60 | 0.005 |
| traspaso_23 | 39.40, −85.90, 60 | 39.40, −85.90, 60 | 0.004 |
| libre_1 | 136.15, −52.46, 40 | 136.15, −52.46, 40 | 0.004 |
| libre_2 | 67.97, −67.28, 20 | 67.97, −67.28, 20 | 0.004 |

Pendiente: correr los dos .m en MATLAB y guardar su salida y su figura.
