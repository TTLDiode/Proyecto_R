# Entrega 2 — Cinemática y MoveIt · Estación 1

**Alimentación y singulación · configuración RRP · Santiago Fernando Machado Sánchez**
Universidad EIA · Robótica y Control Digital · 20 de septiembre de 2026

Todo lo que sigue está medido contra la simulación, no estimado. Los comandos que
reproducen cada cifra están en la última sección.

---

## 1. Tabla de parámetros de Denavit-Hartenberg

Convención estándar, `A_i = Rz(θ_i)·Tz(d_i)·Tx(a_i)·Rx(α_i)`.

| i | θ | d (m) | a (m) | α | tipo | rango |
|---|---|---|---|---|---|---|
| 1 | **θ₁** | 0.173 | 0.180 | 0 | rotacional | −15° … +185° |
| 2 | **θ₂** | −0.053 | 0.140 | π | rotacional | ±135° |
| 3 | 0 | **d₃** | 0 | 0 | prismática | 0 … 84 mm |

Dos decisiones de la tabla que conviene poder defender:

- **α₂ = π** voltea el eje z del sistema 2 hacia abajo. Con eso, `d₃` positiva
  desciende sin tener que meter un signo a mano en las ecuaciones ni en el
  control. Es la misma orientación que el URDF le da al eje de la articulación,
  `axis = (0, 0, −1)`.
- **d₂ = −0.053 m es negativo y no es un error.** Es el salto vertical desde el
  plano del eje de θ₁ hasta la herramienta con la prismática arriba:
  0.097 − 0.115 − 0.035 = −0.053 m. La herramienta cuelga por debajo del brazo.

La cadena resultante es de tipo SCARA: los dos ejes de giro son verticales y
paralelos, y la prismática es vertical. **La gravedad no produce par respecto de
ninguna de las dos rotacionales**, que es la razón por la que esta estación no
necesita frenos ni compensación de gravedad en θ₁ y θ₂.

---

## 2. Cinemática directa

Implementación propia en `station1_control/cinematica.py` y en
`matlab/station1/e1_directa.m`. Las dos construyen las matrices homogéneas a
mano; no se usa ninguna función de *toolbox*, como exige el enunciado.

La forma cerrada equivalente, que es la que se cita en el pitch:

```
x = a₁·cos θ₁ + a₂·cos(θ₁+θ₂)
y = a₁·sen θ₁ + a₂·sen(θ₁+θ₂)
z = d₁ + d₂ − d₃
```

**El plano depende solo de las rotacionales y la altura solo de la prismática.**
Los dos problemas están desacoplados, y de ahí que la inversa salga cerrada.

La orientación alcanzable no es libre: `tool0` gira solidariamente con θ₁+θ₂ y
sus otros dos ejes están fijos. Con tres grados de libertad, fijada la posición
y la rama del codo, la guiñada queda determinada. Esto tiene una consecuencia
directa en la configuración de MoveIt (sección 5).

### Verificación contra el URDF

La cinemática y el URDF describen la misma máquina por dos caminos distintos, de
modo que comparar uno contra otro es una verificación real y no una
comprobación circular. Con el robot en su posición de *homing*
(θ₁ = 180°, θ₂ = 130°, d₃ = 0):

| | x (m) | y (m) | z (m) | guiñada |
|---|---|---|---|---|
| Cinemática directa propia | −0.0900 | −0.1072 | 0.1200 | −50.0° |
| `tf2_echo world station1/tool0` | −0.090 | −0.107 | 0.120 | −49.99° |

---

## 3. Cinemática inversa

Implementación propia y **cerrada**, `cinematica.inversa()` y `e1_inversa.m`.

```
d₃ = d₁ + d₂ − z                                    (desacoplada)
cos θ₂ = (x² + y² − a₁² − a₂²) / (2·a₁·a₂)
θ₂ = ±acos(·)                                        (codo arriba / abajo)
θ₁ = atan2(y,x) − atan2(a₂·sen θ₂, a₁ + a₂·cos θ₂)
```

### Elección de rama

Se prueba primero codo arriba y solo se pasa a codo abajo si la primera no cumple
los límites. El criterio es de **despeje**, no de frecuencia: la rama de codo
arriba aleja el eslabón 2 del pilar del final de carrera, que está 88 mm por
detrás del eje de θ₁.

Del espacio alcanzable del plano:

| | fracción |
|---|---|
| solo admite codo arriba | 18.2 % |
| solo admite codo abajo | 18.2 % |
| admite las dos ramas | 63.6 % |

**El punto de traspaso a la Estación 2 exige codo abajo**, y conviene llegar al
pitch sabiéndolo: la rama de codo arriba pediría θ₁ = −33.2° y el límite inferior
de la articulación está en −15°. Es el único de los seis puntos de tarea que no
se resuelve con la rama preferida.

### Prueba automatizada de ida y vuelta

Es la evidencia que pide la rúbrica. Barre el volumen alcanzable completo y
comprueba `FK(IK(pose)) == pose`; no se limita a puntos conocidos, porque los
fallos de una inversa cerrada aparecen en los bordes y no en el centro.

| | resultado |
|---|---|
| poses ensayadas | 146 955 |
| resueltas | 146 955 (100.0 %) |
| **error máximo de cierre** | **3.26 · 10⁻¹⁶ m** en Python, 3.62 · 10⁻¹⁶ m en MATLAB |
| terminan en codo arriba | 81.8 % |

Los puntos imposibles se rechazan **y por el motivo correcto**, que es lo que
distingue una inversa que razona de una que devuelve `false`:

| caso | motivo devuelto |
|---|---|
| más allá del alcance (0.340, 0, 0.080) | radio fuera del alcance |
| dentro del radio muerto (0.050, 0, 0.080) | fuera de los límites de articulación |
| demasiado alto (0.250, 0, 0.150) | altura fuera de la carrera |
| demasiado bajo (0.250, 0, 0.020) | altura fuera de la carrera |
| detrás, a −60° (0.150, −0.260, 0.080) | fuera de los límites de articulación |

Las mismas comprobaciones existen como aserciones en `test/test_cinematica.py`,
5 pruebas, todas en verde.

### Puntos de tarea

| punto | θ₁ | θ₂ | d₃ | codo |
|---|---|---|---|---|
| Centro de la bandeja | 109.07° | 100.98° | 70.0 mm | arriba |
| Esquina (−,−) | 131.33° | 86.13° | 70.0 mm | arriba |
| Esquina (−,+) | 120.04° | 62.34° | 70.0 mm | arriba |
| Esquina (+,−) | 108.92° | 130.30° | 70.0 mm | arriba |
| Esquina (+,+) | 87.84° | 104.48° | 70.0 mm | arriba |
| Traspaso a Estación 2 | 33.21° | −77.98° | 60.0 mm | **abajo** |

---

## 4. Integración con `ros2_control`

La Entrega 1 dejó el modelo sin control de trayectoria. Aquí se cierra: el URDF
expone las cuatro articulaciones a `ros2_control`, el complemento
`gz_ros2_control` levanta el `controller_manager` dentro de Gazebo y se activan
tres controladores.

| controlador | tipo | articulaciones |
|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | las cuatro |
| `brazo_controller` | JointTrajectoryController | `joint_1`, `joint_2`, `joint_3` |
| `mordaza_controller` | JointTrajectoryController | `gripper_joint` |

**Son dos controladores de trayectoria y no uno.** El brazo son los tres motores
DC, que es lo que planifica MoveIt; la mordaza es el servo. Mezclarlos obligaría
al planificador a tratar el servo como un cuarto grado de libertad, que es justo
lo que la guía no pide.

Se comanda **posición** y no esfuerzo porque los tres actuadores son Pololu 37D
con encoder y su lazo de posición se cierra en el microcontrolador, no en el PC.
Comandar esfuerzo obligaría a simular aquí un lazo que en la máquina real vive en
la Pico.

### Tres cosas que costaron y que hay que saber explicar

**1. Gazebo no encontraba el complemento.** `GZ_SIM_SYSTEM_PLUGIN_PATH` viene
vacía en ROS 2 Jazzy: la biblioteca se instala en `lib/` del prefijo pero nadie
alimenta la variable. El lanzador la compone a partir del prefijo del paquete.
Sin esto el modelo aparece en el mundo sin `controller_manager` y los *spawners*
esperan indefinidamente.

**2. La ganancia del lazo no está donde parece.** `gz_ros2_control` no manda la
posición al motor: calcula el error y lo convierte en velocidad,
`v = −K · update_rate · (posición − consigna)`. La ganancia efectiva es
K · 200 = 100 s⁻¹, constante de tiempo 10 ms, y **esa** es la cifra que hay que
citar, no el 0.5 del archivo. Además la lee su propio nodo, `gz_ros_control`, no
el `controller_manager`: puesta bajo `controller_manager` se acepta sin protestar
y no surte ningún efecto.

**3. `tool0` colgaba de la mordaza.** Estaba unido a `gripper_link`, que gira con
el servo. Dos consecuencias: el punto de trabajo se desplazaba 16.8 mm al abrir
la pinza, y el grupo de planificación tenía cuatro articulaciones en vez de tres.
Ahora cuelga de `link_3`. Con la mordaza cerrada la pose es idéntica a la de la
Entrega 1, de modo que nada de lo entregado antes queda invalidado.

---

## 5. Paquete MoveIt

`station1_moveit_config`, escrito a mano: el Setup Assistant es una herramienta
gráfica, la caja de simulación corre sin escritorio, y para tres articulaciones
el archivo que genera es más largo que el escrito y sus pares de colisión "por
defecto" salen de un muestreo aleatorio que aquí se puede razonar exactamente.

| elemento | valor |
|---|---|
| grupo del brazo | `brazo`, cadena `station1/base_link` → `station1/tool0` |
| grupo de la mordaza | `mordaza`, solo `gripper_joint` |
| poses con nombre | `home`, `reposo`, `bandeja`, `traspaso` |
| solucionador | `kdl_kinematics_plugin/KDLKinematicsPlugin` |
| planificador | OMPL, RRTConnect por defecto |

### `position_only_ik: true` no es un atajo

Es obligatorio y hay que poder explicar por qué. La orientación de la herramienta
no es libre (sección 2): pedirle a un solucionador que case los seis componentes
de una pose con tres articulaciones es pedirle lo imposible, y fallaría en todas
las poses salvo en un conjunto de medida nula. Con `position_only_ik` se le piden
los tres componentes de posición, que es exactamente lo que la máquina puede
cumplir. El registro de `move_group` lo confirma al arrancar:
`Using position only ik`.

### Límites de planificación: medidos, no supuestos

Los límites de `joint_limits.yaml` no se copiaron de la hoja de datos ni se
inventaron. Se derivaron del par disponible y **después se corrigieron contra la
medida**, que es donde está lo interesante.

El par de partida no es el del motor sino el que deja pasar el driver: el Pololu
daría 0.981 N·m en continuo, pero a ese par consume 2.72 A y el TB6612FNG entrega
1.2 A por canal. **Manda el driver: 0.389 N·m.**

Con la inercia del CAD, I = 0.0893 kg·m² para θ₁ con el brazo extendido, sale
α = 4.35 rad/s². Ese número supone que todo el par va a acelerar y no queda nada
para vencer el amortiguamiento ni para que el lazo corrija su propio error.
Medido sobre un giro de 1.324 rad:

| a pedida (rad/s²) | 4.00 | 3.00 | **2.40** | 2.00 | 1.50 |
|---|---|---|---|---|---|
| error máximo (mrad) | 542 | 424 | **10.4** | 10.7 | 5.9 |

Es un **codo**, no una degradación suave: por encima de 2.4 rad/s² el actuador
satura y el eje deja de seguir la consigna. Se declara 2.4, el **55 % del límite
teórico**, y ese 55 % es el margen de par del lazo.

Lo mismo pasa con la velocidad de la prismática, que es la única articulación
cuyo límite es realmente alcanzable:

| v pico (m/s) | 0.0050 | 0.0100 | 0.0160 | **0.0187** | 0.0214 | 0.0267 |
|---|---|---|---|---|---|---|
| error máximo (mm) | 0.04 | 0.08 | 0.34 | **< 1** | 2.21 | 6.78 |

Al 100 % del límite el error llega a 6.8 mm y el controlador aborta la
trayectoria; al 70 % se queda por debajo de 1 mm. **La velocidad de las
rotacionales nunca llega a ser la restricción activa:** alcanzar los 20.944 rad/s
del motor en θ₁ exigiría 4.8 s y 50.4 rad de recorrido, y la articulación entera
mide 3.5 rad. Todo movimiento de esta estación está limitado por aceleración.

---

## 6. Control punto a punto con código propio

`station1_control/punto_a_punto.py`. La cadena completa es:

```
punto cartesiano
  → cinematica.inversa        forma cerrada, sin biblioteca
  → trayectoria.muestrear     perfil quíntico sincronizado
  → FollowJointTrajectory     al brazo_controller
  → lectura del árbol TF      contraste de la pose alcanzada
```

**Perfil quíntico**, `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵`, con todas las articulaciones
recorriendo su trayecto con el mismo polinomio normalizado y llegando a la vez.
Se elige porque anula velocidad y aceleración en los dos extremos: el movimiento
arranca y termina sin escalón de aceleración y, por tanto, sin el pico de *jerk*
que un perfil trapezoidal deja en los bordes. En una máquina con correas dentadas
ese escalón es lo que las hace saltar dientes.

### El acoplamiento dinámico, que es lo que casi rompe la entrega

Un límite de aceleración por articulación supone que las articulaciones son
independientes, y en un brazo planar de dos eslabones **no lo son**. El caso que
lo dejó en evidencia es el paso de la bandeja al punto de traspaso: la bandeja se
alcanza con el codo arriba y el traspaso exige codo abajo, de modo que θ₂ barre
3.12 rad y pasa por la extensión completa. Con el perfil dimensionado solo por
aceleración, θ₁ se quedaba 0.10 rad por detrás de su consigna y el controlador
abortaba.

La cuenta explica por qué: el término de Coriolis que θ₂ induce sobre θ₁ llega a
**0.30 N·m frente a los 0.389 N·m** del presupuesto. `dinamica.py` modela el
brazo como dos cuerpos

```
τ₁ = M₁₁·θ̈₁ + M₁₂·θ̈₂ − h·(2θ̇₁θ̇₂ + θ̇₂²)
τ₂ = M₁₂·θ̈₁ + M₂₂·θ̈₂ + h·θ̇₁²
h(θ₂) = m₂·a₁·r₂·sen θ₂
```

y la duración del movimiento se estira hasta que el par cabe en el presupuesto.
Como con el perfil quíntico tanto el término inercial como el de Coriolis van
como 1/T², el factor de estirado sale de una sola evaluación, sin iterar.

### Resultado

| movimiento | duración | error final |
|---|---|---|
| reposo → bandeja | 7.05 s | 0.000 mm |
| bandeja → traspaso | 2.60 s | 0.000 mm |
| traspaso → bandeja | 2.60 s | 0.001 mm |

### Ciclo completo de la estación

`nodo_estacion.py` ejecuta el ciclo que pide la guía, del `start` al `done`,
con las aproximaciones y la mordaza incluidas:

| tramo | duración |
|---|---|
| aproximación a la pieza | 5.1 s |
| descenso sobre la pieza | 2.1 s |
| cierre de mordaza | 0.6 s |
| ascenso con la pieza | 2.1 s |
| traslado al traspaso | 2.6 s |
| descenso en el traspaso | 2.1 s |
| apertura de mordaza | 0.6 s |
| ascenso sin la pieza | 2.1 s |
| retroceso a reposo | 4.1 s |
| **total** | **21.9 s** |

**Restricción dura de la guía: ≤ 30 s. Quedan 8.1 s de margen, el 27 %.** La
primera versión medía 25.8 s; el detalle de los dos cambios que la bajaron está
en `arquitectura-software-e1.md`, sección 6.

---

## 7. Contraste entre MoveIt y el código propio

Medido sobre los seis puntos de tarea, partiendo siempre del mismo estado.
Tabla completa en `resultados/comparacion_moveit_vs_propia.md`.

### Las dos soluciones coinciden

| punto | máx |Δq| entre las dos | error IK propia | error IK MoveIt |
|---|---|---|---|---|
| bandeja | 2.5 · 10⁻⁵ | 0.000 mm | 0.004 mm |
| esquina (−,−) | 7.0 · 10⁻⁸ | 0.000 mm | 0.000 mm |
| esquina (−,+) | 1.2 · 10⁻⁸ | 0.000 mm | 0.000 mm |
| esquina (+,−) | 6.4 · 10⁻⁶ | 0.000 mm | 0.001 mm |
| esquina (+,+) | 1.8 · 10⁻⁸ | 0.000 mm | 0.000 mm |
| traspaso | 1.3 · 10⁻⁶ | 0.000 mm | 0.000 mm |

Que coincidan no es una casualidad afortunada, es lo que tiene que pasar: las dos
resuelven la misma ecuación. **Lo interesante no es la coincidencia sino el
coste.**

### El coste

| | código propio | MoveIt |
|---|---|---|
| tiempo de la inversa | **15–27 µs** | 2–19 ms |
| naturaleza | cerrada, exacta, siempre converge | iterativa, puede no converger |
| tiempo de planificación | no hay | 27–37 ms |
| error de la pose alcanzada | 0.000 mm | 0.024–0.074 mm |
| comprobación de colisiones | no | sí |

La inversa propia es entre 100 y 1000 veces más rápida. Eso no hace mejor al
código propio: MoveIt está resolviendo un problema más grande, porque comprueba
colisiones y planifica una trayectoria libre de ellas. Es la comparación honesta.

### Duración y par: la diferencia real

| punto | duración propia | par propia | duración MoveIt | par MoveIt | % del presupuesto |
|---|---|---|---|---|---|
| bandeja | 7.02 s | 0.002 N·m | 3.81 s | 0.084 N·m | 22 % |
| esquina (−,−) | 7.02 s | 0.004 N·m | 3.83 s | 0.127 N·m | 33 % |
| esquina (−,+) | 7.02 s | 0.003 N·m | 3.80 s | 0.110 N·m | 28 % |
| esquina (+,−) | 7.02 s | 0.003 N·m | 3.80 s | 0.113 N·m | 29 % |
| esquina (+,+) | 7.02 s | 0.000 N·m | 3.80 s | 0.013 N·m | 3 % |
| traspaso | 6.02 s | 0.031 N·m | 3.34 s | 0.268 N·m | **69 %** |

**MoveIt tarda la mitad. La razón no es que planifique mejor, es la forma del
perfil.** El quíntico tiene una relación pico/media de velocidad de 1.875, de
modo que para el mismo límite de velocidad tarda 1.875 veces más. La medida lo
confirma: 7.05 / 3.81 = 1.85. Ese es el precio de la suavidad C².

Y el par lo paga: la reparametrización de MoveIt usa los límites por articulación
de `joint_limits.yaml`, que son independientes entre sí y **no ven el
acoplamiento**. Evaluada con el modelo dinámico, su trayectoria al punto de
traspaso exige el 69 % del par disponible, contra el 8 % que exige la propia.
Cabe en el presupuesto, pero por poco y sin saberlo.

**Conclusión operativa para la Entrega 3:** para el ciclo de la línea se usa el
código propio. El margen de tiempo lo hay de sobra — 13.8 s contra un límite de
30 — y a cambio se gana una trayectoria suave y un par conocido. MoveIt se queda
para lo que hace mejor: planificar alrededor de obstáculos cuando la escena deje
de estar despejada.

---

## 8. Cómo reproducir cada cifra

```bash
# Entorno
source /opt/ros/jazzy/setup.bash
source ~/proyecto-r-linea-simulada/install/setup.bash
source ~/E2_propuesta/install/setup.bash

# Cinemática, sin simulador
ros2 run station1_control prueba_cinematica         # barrido e informe
cd src/station1_control && python3 -m pytest test    # 5 pruebas

# Verificación en MATLAB
cd ~/proyecto-r-linea-simulada/matlab/station1
~/MATLAB/R2026a/bin/matlab -batch "e1_prueba_cinematica"

# Simulación con control
ros2 launch station1_description simulacion.launch.py gui:=false
ros2 control list_controllers

# Movimiento punto a punto con código propio
ros2 run station1_control punto_a_punto --pose bandeja
ros2 run station1_control punto_a_punto --ciclo       # mide el tiempo de ciclo

# MoveIt
ros2 launch station1_moveit_config move_group.launch.py rviz:=true
ros2 run station1_control comparar_moveit             # genera la tabla
```

> `colcon test` no descubre las pruebas de este paquete: ejecuta pytest en el
> directorio de compilación, donde no hay carpeta `test`. Se corren con
> `python3 -m pytest test` desde `src/station1_control`, y pasan las cinco.

## 9. Evidencia

| archivo | contenido |
|---|---|
| `evidencia/rviz_movimiento.png` | RViz con el grupo `brazo`, escena de planificación y estado actual |
| `evidencia/movimiento_punto_a_punto.gif` | secuencia del movimiento reposo → bandeja |
| `resultados/comparacion_moveit_vs_propia.md` | tabla generada por `comparar_moveit` |
| `resultados/arquitectura-software-e1.md` | punto 4.4 de la guía: nodos, tópicos, servicios, diagrama de flujo y protocolo de línea |

---

## 10. Lo que queda abierto para la Entrega 3

Se dice aquí y no se esconde, porque son las preguntas que el profesor puede
hacer:

1. **La pieza todavía no existe en el mundo.** El ciclo mueve el brazo y anima la
   mordaza, pero no hay un objeto que agarrar. La retención por acople
   cinemático de ADR-002 (`DetachableJoint` en Gazebo Harmonic) es trabajo de la
   Entrega 3.
2. **El tiempo de ciclo medido es sin pieza.** Añadirla cambia la inercia del
   carro en 20 g sobre 221 g, es decir un 9 %, que afecta a la prismática y no a
   las rotacionales. El margen de 8 s absorbe eso de sobra, pero hay que volver a
   medirlo con la pieza puesta.
3. **`colcon test` no descubre las pruebas** por cómo ejecuta pytest en el
   directorio de compilación. Las pruebas existen y pasan; el descubrimiento
   automático queda pendiente.
