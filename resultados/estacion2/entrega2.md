# Entrega 2: Cinemática y MoveIt · Estación 2

**TRÍADA · clasificación por visión · configuración RRP (SCARA)**
Universidad EIA · Robótica y Control Digital · octubre de 2026

Todas las cifras están medidas contra la simulación o calculadas con el código
del repositorio. Los comandos que las reproducen están en la última sección.

---

## 1. Tabla de Denavit-Hartenberg

Convención estándar, `A_i = Rz(θ_i)·Tz(d_i)·Tx(a_i)·Rx(α_i)`.

| i | θ | d (m) | a (m) | α | tipo | rango |
|---|---|---|---|---|---|---|
| 1 | **θ₁** | 0.114 | 0.160 | 0 | rotacional | −30° … +210° |
| 2 | **θ₂** | 0.006 | 0.140 | π | rotacional | ±135° |
| 3 | 0 | **d₃** | 0 | 0 | prismática | 0 … 80 mm |

- **d₁ = 0.114 m** es la altura del eje de θ₁ sobre el suelo, tomada del CAD.
- **d₂ = 0.006 m** lleva del plano de θ₁ a la herramienta con d₃ = 0: el
  eslabón 2 está 42 mm por encima y el carro de la prismática cuelga 36 mm por
  debajo de él (0.114 + 0.042 − 0.036 = 0.120 m).
- **α₂ = π** voltea el eje z del sistema 2 hacia abajo, de modo que d₃ positiva
  desciende, igual que el eje `(0, 0, −1)` de la articulación en el URDF.

Es una cadena SCARA: los dos ejes de giro son verticales y paralelos. La
gravedad no produce par en θ₁ ni en θ₂; solo la prismática la soporta (44 N de
empuje del husillo Tr8 contra unos 3 N de peso).

Alcance: radio de 116.3 a 300.0 mm y tool0 entre z = 40 y 120 mm, dentro del
envolvente de la guía (320 ± 20 mm).

---

## 2. Cinemática directa

Implementación propia en `src/station2_control/station2_control/cinematica.py`
y `matlab/station2/e2_directa.m`. Las matrices homogéneas se construyen a mano,
sin funciones de *toolbox*.

```
x = a₁·cos θ₁ + a₂·cos(θ₁+θ₂)
y = a₁·sen θ₁ + a₂·sen(θ₁+θ₂)
z = d₁ + d₂ − d₃
```

El plano depende solo de las rotacionales y la altura solo de la prismática.
La orientación de tool0 no es libre: gira con θ₁+θ₂.

La cámara (Camera Module 3 Wide) va bajo el eslabón 2, a 70 mm del codo y 9 mm
a un lado. Su posición tiene la misma forma, con la segunda barra de 70.6 mm
girada 7.3°:

```
x_c = a₁·cos θ₁ + 0.0706·cos(θ₁+θ₂+7.3°)
y_c = a₁·sen θ₁ + 0.0706·sen(θ₁+θ₂+7.3°)
z_c = 0.151 m (fija)
```

---

## 3. Cinemática inversa, en forma cerrada

`cinematica.inversa()` y `matlab/station2/e2_inversa.m`:

```
d₃      = d₁ + d₂ − z
cos θ₂  = (x² + y² − a₁² − a₂²) / (2·a₁·a₂)
θ₂      = ± acos(...)                          codo arriba / codo abajo
θ₁      = atan2(y, x) − atan2(a₂·sen θ₂, a₁ + a₂·cos θ₂)
```

**Elección de rama.** Es la particularidad de esta estación: los dos traspasos
caen en ramas distintas.

| punto (world) | en la base | θ₁ | θ₂ | d₃ | rama | la otra rama |
|---|---|---|---|---|---|---|
| traspaso 1-2 (0.250, 0, 0.060) | (−0.250, 0) | 148.90° | 67.28° | 60 mm | arriba | θ₁ = 211.1°, fuera de límite |
| puesto de inspección (0.500, 0.200, 0.060) | (0, 0.200) | 45.95° | 96.67° | 60 mm | arriba | también válida |
| traspaso 2-3 (0.720, 0, 0.060) | (0.220, 0) | 39.40° | −85.90° | 60 mm | abajo | θ₁ = −39.4°, fuera de límite |
| cámara sobre el puesto | (0, 0.200) | 71.29° | 58.02° | 0 | arriba | |

La lente no llega encima del traspaso 1-2: está a 250 mm de la base y la lente
alcanza como mucho 230.6 mm. Por eso la estación deja la pieza en un **puesto de
inspección** a 200 mm, en +y, donde llegan tanto la lente como la mordaza.

### Verificación

| prueba | resultado |
|---|---|
| ida y vuelta directa → inversa, Python, rejilla articular | 18 300 de 18 300, peor cierre 3.12·10⁻¹⁶ m |
| ida y vuelta, MATLAB/Octave, rejilla de 4° y 10 mm, dos ramas | 64 926 resueltas, peor cierre 2.56·10⁻¹⁶ m |
| puntos imposibles (5 casos) | los 5 rechazados con el motivo correcto |
| pruebas unitarias (`pytest`) | 6 de 6 |
| tool0 en Gazebo contra la inversa propia, en los traspasos | 0.00 mm (resolución del registro 0.01 mm) |

Salidas completas en `resultados/estacion2/matlab_prueba_cinematica.txt`.

---

## 4. ros2_control

`src/station2_description/urdf/station2.ros2_control.xacro` y
`config/station2_controllers.yaml`:

- `gz_ros2_control/GazeboSimSystem`, interfaz de mando de posición y estados de
  posición, velocidad y esfuerzo.
- `brazo_controller` (JointTrajectoryController: joint_1, joint_2, joint_3) y
  `mordaza_controller` (los dos dedos), con `joint_state_broadcaster`.
- Todo bajo el espacio de nombres `/station2`, para convivir con las otras dos
  estaciones en el mismo Gazebo.

Límites de la máquina, del Pololu 37D 50:1 (20.944 rad/s en el eje) por cada
transmisión y del par que deja pasar el TB6612FNG:

| articulación | transmisión | v máx | a usada | par / fuerza |
|---|---|---|---|---|
| θ₁ | correa 3:1 | 6.98 rad/s | 4.0 rad/s² | 1.167 N·m |
| θ₂ | directa 1:1 | 20.94 rad/s | 4.0 rad/s² | 0.389 N·m |
| d₃ | 1:3 y husillo Tr8 | 0.080 m/s | 0.5 m/s² | 44 N |

---

## 5. Punto a punto con código propio

`ros2 run station2_control punto_a_punto --pose traspaso_12` (o `--ciclo`, o
tres coordenadas):

punto cartesiano en world → `a_base` + `inversa` cerrada → perfil quíntico
sincronizado (`trayectoria.py`) → `FollowJointTrajectory` al `brazo_controller`
→ lectura de tool0 en el árbol TF y error contra el punto pedido.

El perfil `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵` anula velocidad y aceleración en los
extremos. La duración es la mínima que respeta los límites de la tabla
anterior, con el 70 % de la velocidad como reserva para el lazo de posición.

---

## 6. MoveIt y comparación

Paquete `src/station2_moveit_config`: SRDF escrito a mano (grupo `brazo` como
cadena base_link → tool0, grupo `mordaza`, poses con nombre, colisiones
adyacentes desactivadas), KDL con `position_only_ik` (3 GDL: la orientación no
es libre), OMPL con RRTConnect y `moveit_simple_controller_manager` hacia los
controladores de Gazebo. move_group corre en `/station2`.

`ros2 run station2_control comparar_moveit` lleva el brazo a cada punto por las
dos vías, partiendo siempre del reposo:

| punto | q propia (°, °, mm) | q MoveIt / KDL | máx Δq | error final propia | error final MoveIt |
|---|---|---|---|---|---|
| traspaso 1-2 | 148.90, 67.28, 60.0 (arriba) | 148.90, 67.29, 60.0 (arriba) | 2.8·10⁻⁵ | 0.001 mm | 0.021 mm |
| inspección | 45.95, 96.67, 60.0 (arriba) | 45.95, 96.67, 60.0 (arriba) | 7.7·10⁻⁷ | 0.001 mm | 0.095 mm |
| traspaso 2-3 | 39.40, −85.90, 60.0 (abajo) | 39.40, −85.90, 60.0 (abajo) | 6.2·10⁻⁸ | 0.000 mm | 0.166 mm |
| libre 1 (0.40, 0.25, 0.08) | 87.45, 52.46, 40.0 (arriba) | 136.15, −52.46, 40.0 (abajo) | 7.8·10⁻⁶ * | 0.000 mm | 0.016 mm |
| libre 2 (0.70, 0.15, 0.10) | 5.77, 67.28, 20.0 (arriba) | 67.97, −67.28, 20.0 (abajo) | 2.1·10⁻⁷ * | 0.002 mm | 0.078 mm |

\* Δq contra la solución propia de la misma rama.

| punto | t IK propia | t IK MoveIt | t plan MoveIt | duración propia | duración MoveIt |
|---|---|---|---|---|---|
| traspaso 1-2 | 55.6 µs | 25.6 ms | 212.6 ms | 2.01 s | 1.35 s |
| inspección | 48.4 µs | 9.1 ms | 51.6 ms | 2.01 s | 1.46 s |
| traspaso 2-3 | 40.3 µs | 7.5 ms | 28.7 ms | 2.01 s | 1.42 s |
| libre 1 | 92.5 µs | 4.2 ms | 28.5 ms | 1.34 s | 1.03 s |
| libre 2 | 36.4 µs | 7.4 ms | 22.4 ms | 1.46 s | 1.08 s |

Lectura:

- **Las dos vías llegan al mismo punto**, con errores finales por debajo de
  0.2 mm, cien veces por debajo de la tolerancia de 20 mm de la guía.
- **La inversa cerrada es unas 100 a 500 veces más rápida** que KDL y siempre
  da respuesta; KDL es iterativo.
- **KDL no elige rama**: en los puntos libres converge a codo abajo, la rama más
  cercana a la semilla, mientras que la inversa propia prueba primero codo
  arriba. En los traspasos coinciden porque solo una rama cabe en los límites.
  Para esta estación eso importa: la rama se elige a propósito en cada punto
  del ciclo.
- **MoveIt es más rápido en la ejecución** (parametrización temporal óptima con
  los mismos límites) y el perfil propio es más conservador: el quíntico no
  tiene tramo a velocidad constante y deja un 30 % de reserva de velocidad.
  A cambio, el quíntico no tiene escalones de aceleración, que en una máquina
  de correas es lo que hace saltar dientes.

Tabla generada: `resultados/estacion2/comparacion_moveit_vs_propia.md`.

---

## 7. Cómo reproducir

```bash
colcon build --packages-up-to station2_control station2_moveit_config
source install/setup.bash

# cinemática
ros2 run station2_control prueba_cinematica
cd matlab/station2 && octave --eval e2_prueba_cinematica   # o MATLAB

# simulación, punto a punto y MoveIt
ros2 launch station2_description simulacion.launch.py
ros2 run station2_control punto_a_punto --ciclo
ros2 launch station2_moveit_config move_group.launch.py rviz:=true
ros2 run station2_control comparar_moveit --salida resultados/estacion2

# modelo en RViz2
ros2 launch station2_description display.launch.py
```
