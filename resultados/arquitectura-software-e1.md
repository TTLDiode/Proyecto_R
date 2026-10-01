# Arquitectura de software de la Estación 1

Universidad EIA · Robótica y Control Digital · Entrega 2

Responde al punto 4.4 de la guía: arquitectura de nodos, tópicos y servicios,
diagrama de flujo, descripción del nodo de control y protocolo de comunicación
de inicio y fin con las estaciones adyacentes.

---

## 1. Nodos

| nodo | paquete | papel |
|---|---|---|
| `gz sim` + `gz_ros2_control` | — | simulador. Aloja el `controller_manager` dentro del propio proceso de Gazebo |
| `controller_manager` | `controller_manager` | carga y activa los controladores; vive dentro del simulador |
| `robot_state_publisher` | `robot_state_publisher` | publica el árbol TF a partir del URDF y de `/joint_states` |
| `parameter_bridge` | `ros_gz_bridge` | puente del reloj de simulación, `/clock` |
| `move_group` | `moveit_ros_move_group` | planificación, cinemática inversa numérica y ejecución supervisada |
| **`station1_punto_a_punto`** | **`station1_control`** | **control de trayectoria con cinemática propia** |
| **`nodo_estacion`** | **`station1_control`** | **máquina de estados de la estación y contrato de línea** |

Los dos últimos son el trabajo propio; los demás son infraestructura.

## 2. Tópicos y servicios

### Internos de la estación

| endpoint | tipo | dirección |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | del simulador, 200 Hz |
| `/robot_description` | `std_msgs/String` | URDF expandido |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | árbol de transformadas |
| `/brazo_controller/follow_joint_trajectory` | acción `FollowJointTrajectory` | entrada del controlador del brazo |
| `/mordaza_controller/follow_joint_trajectory` | acción `FollowJointTrajectory` | entrada del controlador del servo |
| `/compute_ik` | servicio `GetPositionIK` | cinemática inversa de MoveIt |
| `/move_action` | acción `MoveGroup` | planificación de MoveIt |
| `/execute_trajectory` | acción `ExecuteTrajectory` | ejecución supervisada por MoveIt |

### De línea, según `docs/03-contrato-de-interfaz.md`

| endpoint | tipo | dirección | significado |
|---|---|---|---|
| `/line/station1/start` | servicio `StartTask` | **entrada** | arranca un ciclo. Responde `accepted=false` si hay uno en curso |
| `/line/station1/done` | tópico `HandoffEvent` | **salida** | se publica **una sola vez** al terminar, con la pieza en su pose de salida |
| `/line/station1/state` | tópico `StationState` | **salida** | latido a 2 Hz, siempre activo mientras el nodo viva |

**El latido es la parte que más se subestima.** El orquestador distingue una
estación ocupada de una estación caída por la ausencia del latido, no por el
silencio en `/done`. Sin él, un nodo muerto y un nodo trabajando se parecen
demasiado.

## 3. Diagrama de flujo del ciclo

```
        ┌──────────────────────────────────────────────────────────┐
        │  IDLE  ── latido a 2 Hz ──►  /line/station1/state        │
        └────────────────────────┬─────────────────────────────────┘
                                 │  srv /line/station1/start
                                 │  (rechaza si ya está ocupada)
                                 ▼
        ┌──────────────────────────────────────────────────────────┐
        │  BUSY                                                    │
        │                                                          │
        │   abrir mordaza                                          │
        │        │                                                 │
        │        ▼                                                 │
        │   aproximación:  IK(pieza + 20 mm)  → perfil → brazo     │
        │        │                                                 │
        │        ▼                                                 │
        │   descenso:      IK(pieza)          → solo prismática    │
        │        │                                                 │
        │        ▼                                                 │
        │   cerrar mordaza                                         │
        │        │                                                 │
        │        ▼                                                 │
        │   ascenso        ──►  traslado al traspaso               │
        │                            │                             │
        │                            ▼                             │
        │                       descenso → abrir mordaza → ascenso │
        │                            │                             │
        │                            ▼                             │
        │                       retroceso a reposo                 │
        └────────────────────────┬─────────────────────────────────┘
                                 │  publica HandoffEvent
                                 ▼  en /line/station1/done
        ┌──────────────────────────────────────────────────────────┐
        │  DONE  ──►  IDLE                                         │
        └──────────────────────────────────────────────────────────┘

        Cualquier excepción en el ciclo  ──►  ERROR, con el motivo en
        el campo `detail` del latido. La estación no se reinicia sola.
```

**Las aproximaciones son siempre verticales**, nunca en diagonal. Con una cadena
RRP eso sale gratis: el descenso es la prismática sola, con las dos rotacionales
quietas, de modo que el movimiento de bajada no puede arrastrar la pieza
lateralmente. Es la ventaja práctica de la configuración elegida y es un buen
argumento para defender la elección RRP en una estación de *pick and place*.

## 4. El nodo de control de trayectoria

`station1_control/punto_a_punto.py`. La cadena, de un punto cartesiano al
movimiento real:

```
punto (x, y, z) en frame world
   │
   ├─► cinematica.inversa()      forma cerrada, sin biblioteca de robótica
   │      devuelve (θ₁, θ₂, d₃) o el motivo por el que no es alcanzable
   │
   ├─► trayectoria.duracion()    T mínimo que cabe en los límites
   │      · cota por velocidad y aceleración de cada articulación
   │      · cota por presupuesto de par del modelo dinámico (acoplamiento)
   │
   ├─► trayectoria.muestrear()   perfil quíntico sincronizado, 50 Hz
   │      s(τ) = 10τ³ − 15τ⁴ + 6τ⁵, velocidad y aceleración nulas en los extremos
   │
   ├─► FollowJointTrajectory     al brazo_controller
   │
   └─► lectura del árbol TF      contraste de la pose alcanzada contra la pedida
```

El último paso no es decorativo: es el único que cierra el lazo contra el
simulador y no contra la propia cinemática, que siempre se da la razón a sí
misma.

**Los módulos y qué responde cada uno:**

| módulo | responde a |
|---|---|
| `cinematica.py` | ¿a qué posición articular corresponde este punto? |
| `dinamica.py` | ¿cuánto par exige este movimiento? |
| `trayectoria.py` | ¿cómo llegar, y en cuánto tiempo? |
| `punto_a_punto.py` | ejecutar y comprobar |
| `nodo_estacion.py` | cuándo, y a quién avisar |

## 5. Protocolo con las estaciones adyacentes

La Estación 1 es la **fuente** de la línea: no tiene estación aguas arriba, de
modo que su única frontera es con la Estación 2.

**Al inicio.** Recibe `StartTask` en `/line/station1/start`. El campo
`input.pose` indica dónde está la pieza en la bandeja; si llega en cero, se usa
el centro de la bandeja. Esto es deliberado: la guía dice que las posiciones en
la bandeja *no están totalmente definidas*, de modo que quien pide el ciclo puede
decir dónde está la pieza, y la estación funciona igualmente sola para probarla.

**Al final.** Publica `HandoffEvent` en `/line/station1/done` con la pieza en el
punto de traspaso, `(0.250, 0.000, 0.060)` en frame `world`, tomado de
`line_poses.yaml`. El campo `category` sale vacío: **la Estación 1 no clasifica**,
eso es competencia de la Estación 2.

**Las tres reglas que no se negocian:**

1. `/done` se publica **una sola vez** por ciclo. Si se publicase dos veces, la
   Estación 2 arrancaría dos veces con la misma pieza.
2. Un `start` recibido mientras hay un ciclo en curso se **rechaza** con
   `accepted=false`. El rechazo no es un error: el orquestador reintenta. Lo que
   no puede ocurrir es que dos ciclos se solapen sobre la misma máquina.
3. La estación **no se reinicia sola** tras un `ERROR`. Un brazo que reintenta
   por su cuenta después de un fallo que no entiende es más peligroso que un
   brazo parado.

## 6. Tiempo de ciclo medido

Restricción dura de la guía para esta estación: **≤ 30 s**. Medido sobre el ciclo
completo, del `start` al `done`:

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

Quedan **8.1 s de margen, el 27 %**.

Dónde se va el tiempo y qué se hizo con ello: la prismática es la articulación
lenta, 0.0187 m/s de pico, y los cuatro tramos verticales son suyos. La primera
versión del ciclo medía 25.8 s; bajó a 21.9 s con dos cambios, y los dos tienen
justificación:

- **La aproximación pasó de 30 a 20 mm.** Cada 10 mm de aproximación cuestan un
  segundo, y se recorren cuatro veces. La pieza mide 12 mm de alto, de modo que a
  20 mm quedan 8 mm de holgura sobre ella al desplazarse en horizontal.
- **Se quitó el paso por reposo antes de recoger**, que costaba 4.6 s. El ciclo
  anterior ya deja el brazo ahí, y si no, el movimiento a la aproximación es
  igualmente válido desde cualquier pose alcanzable. El retroceso a reposo del
  final **sí se conserva**: deja el brazo fuera del punto de traspaso, que es
  justo donde la Estación 2 va a meter el suyo.
