# Entrega 3 — Infraestructura de integración: estado y plan

Universidad EIA · Robótica y Control Digital · 21 de septiembre de 2026

El plan maestro dice que en la Entrega 3 **la integración va primero, no al
final**, porque es el único punto de fallo compartido y vale el 40 % de la nota.
Este documento registra qué parte de esa infraestructura ya funciona, medida, y
qué falta.

---

## 1. Lo que ya corre

### 1.1 La línea completa con los tres nodos simulados

```bash
ros2 launch line_bringup line.launch.py n_parts:=2
```

| pieza | categoría asignada | tiempo de línea |
|---|---|---|
| 1 | B | 9.0 s |
| 2 | C | 9.0 s |

Las tres estaciones simuladas encadenan correctamente, el orquestador asigna
identificadores de pieza, espera el evento de fin de cada una y pasa el
resultado a la siguiente. **Esto es el "seguro" del gate G10**: existe una demo
de la línea completa pase lo que pase, y estaba previsto para el 18 de octubre.

### 1.2 La línea con la Estación 1 real

```bash
ros2 launch station1_description simulacion.launch.py gui:=false   # Gazebo + control
ros2 run station1_control nodo_estacion                            # estación real
ros2 launch line_bringup line.launch.py mock_stations:=station2,station3 n_parts:=2
```

**El manipulador real de la Estación 1 se integra en la línea sin tocar ni el
orquestador ni los otros dos nodos.** Basta sacar `station1` del argumento
`mock_stations`. Esto es el gate G11 para esta estación, previsto para el 25 de
octubre.

Es la prueba de que el contrato de interfaz de ADR-004 hace lo que prometía: la
estación es una caja negra, y el nodo simulado y el real son intercambiables
porque hablan el mismo protocolo.

### 1.3 La línea moviendo materia, no solo información

Esto era el riesgo alto de la versión anterior de este documento y **ya está
resuelto**. El mundo tiene ahora bandeja de entrada, soporte de traspaso y una
pieza de 20 × 20 × 12 mm y 20 g, y la Estación 1 la coge, la transporta y la
deposita.

```bash
./probar_linea_con_pieza.sh        # E1 real con pieza + E2 y E3 simuladas
```

| pieza | ciclo real de la E1 | tiempo de línea | categoría | pieza depositada en |
|---|---|---|---|---|
| 1 | 22.6 s | 28.6 s | B | (0.249992, −0.000007, 0.048000) |
| 2 | 22.6 s | 28.6 s | C | (0.249992, −0.000008, 0.048000) |

Objetivo: (0.250, 0.000, 0.048). **Error de colocación 0.011 mm**, y las dos
piezas caen prácticamente en el mismo punto: la repetibilidad entre ciclos es de
1 µm. La restricción de la guía es 20 mm.

El coste de llevar la pieza es **0.7 s sobre los 21.9 s del ciclo en vacío**, un
3.2 %. Estaba previsto que los 20 g sobre los 221 g del carro afectaran solo a
la prismática, y eso es lo que se mide. Quedan 7.4 s de margen sobre el límite
de 30 s.

Entre ciclo y ciclo el guion repone la pieza en la bandeja. No es trampa: en la
línea terminada es la Estación 2 la que se la lleva del punto de traspaso y la
bandeja se repone desde fuera. Mientras esas dos estaciones no existan, reponer
la pieza es lo que mantiene la corrida honesta.

#### Dos cosas que costaron encontrar y conviene tener escritas

**`DetachableJoint` nace acoplado.** La documentación del complemento da a
entender que espera una petición en `attach_topic` para unir los cuerpos, pero
no es así: los une al cargar el mundo. Se comprobó dejando correr el brazo sin
pedir ningún acople: la pieza se movió de (−0.180, 0.100, 0.0380) a
(−0.166, 0.099, 0.0385), arrastrada. La solución es publicar un desacople único
al arrancar, en el propio `simulacion.launch.py`, de modo que el estado inicial
sea el que el código cree que es.

**La mordaza al abrirse baja por debajo del efector.** Abrir gira el bloque de
35 mm 0.5 rad sobre el eje X, y su esquina inferior queda 2.9 mm más abajo que
`tool0`. Como el descenso a por la pieza se hace con la mordaza abierta, esa
esquina golpeaba la pieza y la lanzaba fuera de la bandeja. Se resolvió fijando
las alturas de bandeja y soporte para que **la cara alta de la pieza quede 6 mm
por debajo del punto de tarea**, con lo que quedan 3.1 mm de holgura incluso con
la mordaza abierta del todo.

Es una simplificación de modelado que conviene decir en la sustentación: la
mordaza sujeta desde arriba y no por los lados. La retención la hace el acople
cinemático de ADR-002, no el contacto, así que la holgura no cambia el
comportamiento.

---

## 2. Un hallazgo que afecta al contrato de interfaz

La guía pide que la Estación 1 entregue el componente "en pose conocida". Pose
incluye orientación, y con tres grados de libertad **la orientación no es una
variable libre**: en una cadena RRP la guiñada de la herramienta vale θ₁ + θ₂ y
queda determinada en cuanto se fija la posición.

La pieza se coge rígida, de modo que su giro al llegar es

```
giro_final = giro_inicial + (guiñada_en_traspaso − guiñada_en_recogida)
```

La guiñada en el traspaso siempre vale −44.77°, porque el punto es fijo. La de
recogida no, porque la guía dice expresamente que las posiciones en la bandeja
no están totalmente definidas:

| punto de recogida | guiñada | giro que sufre la pieza |
|---|---|---|
| centro de la bandeja | 210.06° | 105.18° |
| esquina (−,−) | 217.47° | 97.77° |
| esquina (−,+) | 182.38° | 132.85° |
| esquina (+,−) | 239.22° | 76.02° |
| esquina (+,+) | 192.32° | 122.91° |

El modelo predice 105.18° para la recogida en el centro y la simulación mide
105.17°. **La dispersión es de 56.8°** según de dónde se recoja.

Es decir: la Estación 1 entrega una **posición** conocida y repetible a 0.011 mm,
pero una **orientación** que depende de dónde estuviera la pieza en la bandeja.
No es un defecto de implementación y no se puede corregir con más ajuste; es
una consecuencia de tener tres grados de libertad. Hay tres salidas y conviene
elegir una **antes** de la integración, no durante:

1. Que la pieza sea indiferente al giro, por ejemplo cuadrada. Es lo que asume
   hoy el modelo, con una pieza de 20 × 20 mm.
2. Que la Estación 2 la mida con su cámara, que ya lleva para inspección.
3. Que el soporte del traspaso tenga un alojamiento que la oriente al
   depositarla, que es lo que se hace en una línea real.

La opción 3 es la más robusta y la más barata de simular, y no obliga a la
Estación 2 a nada.

---

## 3. Lo que falta, por orden de riesgo

### 3.1 Las otras dos estaciones (riesgo compartido)

No dependen de mí y no puedo adelantarlas. Lo que sí está de mi lado:

- Los nodos simulados cubren su ausencia indefinidamente. La línea corre hoy sin
  ellas.
- El punto de traspaso `(0.250, 0.000, 0.060)` está congelado en
  `line_poses.yaml` y la Estación 1 deposita ahí con 0.011 mm de error medido,
  con una pieza real en el mundo. Lo que la Estación 2 reciba está definido y
  verificado, salvo la orientación de §2.

### 3.2 Un solo mundo con las tres estaciones (riesgo medio)

El mundo `line.sdf` ya tiene suelo, luz, bandeja, soporte de traspaso, pieza y
vista fija de cámara, pero la Estación 1 sigue siendo la única que se spawnea.
El gate G9, previsto para el 11 de octubre, pide las tres a la vez con el árbol
TF limpio. La infraestructura para ello ya está:

- Poses base fijadas en `line_poses.yaml`, separación de 0.500 m.
- Prefijo de frames por estación, `station1/`, según ADR-005. La Estación 1 ya
  lo usa en todos sus eslabones y articulaciones.
- El único riesgo real es que las otras dos no prefijen sus frames. **Conviene
  avisarlo ahora**, no el 11 de octubre: dos estaciones con un eslabón llamado
  `base_link` producen un árbol TF corrupto y el fallo aparece lejos de su
  causa.

### 3.3 Verificaciones de la guía

| restricción | estación | estado |
|---|---|---|
| ciclo ≤ 30 s | E1 | **medido: 22.6 s con pieza**, margen de 7.4 s |
| pose de entrega conocida | E1 | **posición sí, 0.011 mm; orientación varía 56.8°** (§2) |
| precisión < 20 mm | E2 | no depende de mí |
| 10 unidades por caja | E3 | no depende de mí |

---

## 4. Orden de trabajo restante

1. ~~Pieza y acople en la Estación 1 sola.~~ **Hecho**, §1.3.
2. **Mundo único con las tres bases**, aunque las otras dos estén inmóviles o
   sean cajas. Da el gate G9 y destapa cualquier colisión de nombres TF
   temprano.
3. **Sustitución de nodos simulados uno a uno**, en el orden del plan:
   E1 (hecho) → E3 → E2. La más riesgosa la última, cuando la línea ya es
   estable.
4. **Grabar el vídeo el 1 de noviembre con lo que haya.** Un vídeo mediocre
   entregado vale infinitamente más que uno excelente que no se grabó porque la
   simulación se rompió el día anterior.

---

## 5. Lo que hay que comunicar al equipo esta semana

El contrato de interfaz solo sirve si las tres estaciones lo cumplen. La
Estación 1 lo cumple y está verificado. Tres cosas que confirmar con los otros
dos integrantes **antes** del 11 de octubre, no después:

1. **Que prefijan sus frames TF** con `station2/` y `station3/` (ADR-005).
2. **Que implementan los tres endpoints** con los nombres exactos del contrato:
   `/line/stationN/start`, `/line/stationN/done`, `/line/stationN/state`.
3. **Qué hacemos con la orientación del traspaso** (§2). Es la única decisión
   nueva, y es de los tres, no mía: afecta a la geometría de la pieza y a lo que
   la Estación 2 tiene que hacer al recogerla.

Si cualquiera de las tres falla, la integración no ocurre, y el momento de
descubrirlo es ahora que hay siete semanas, no en la última.
