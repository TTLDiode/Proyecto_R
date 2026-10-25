# Estación 2 · estado de las entregas

Actualizado el 25/10/2026. Requisitos según el índice de la guía (Notion,
«Índice de requisitos de la guía»). Rutas en las ramas `estacion2-cad` (CAD,
eléctrico, Notion) y `estacion2-ros` (ROS 2, resultados).

## Entrega 1 · CAD, circuitos y modelo en Gazebo

| Requisito | Evidencia | Estado |
|---|---|---|
| CAD completo: vistas de ensamble y explosionadas | `docs/estacion2/vistas/`, planos A3-001, A3-002, A2-003-1/2 | Cumplido |
| Planos de piezas no comerciales con material y fabricación | `docs/estacion2/planos/planos_estacion2.pdf` (37 hojas) | Cumplido |
| Fichas técnicas de piezas comerciales | `docs/estacion2/fichas-tecnicas/` (18 PDF e índice) | Cumplido |
| BOM: ítem, nombre, material, referencia, cantidad | `docs/estacion2/bom.md`, `bom.csv` (140 ítems) | Cumplido |
| Material real de cada pieza | Notion Mecánica §7, BOM | Cumplido |
| Montaje de actuadores con dimensiones reales | CAD v5.7, Notion Mecánica §8 | Cumplido |
| Esquemáticos (micro, TB6612FNG, final de carrera) | `docs/estacion2/electrico/esquematico_pcb_control.pdf` | Cumplido |
| Separación potencia/lógica y etiquetado de señales | `diagrama_bloques.pdf`, tabla de pines | Cumplido |
| Diagrama de cableado | `docs/estacion2/electrico/diagrama_cableado.pdf` | Cumplido |
| URDF/Xacro del CAD con inercias y colisiones | `src/station2_description/`, mallas del CAD con color | Cumplido |
| RViz2 con árbol TF correcto | `resultados/estacion2/entrega1/` | Cumplido |
| Mundo de Gazebo y spawn del robot | `simulacion.launch.py`, `entrega1/gazebo_spawn.png` | Cumplido |
| Espacio de trabajo y articulaciones con rango | Notion Diseño §1–2, `prueba_cinematica` | Cumplido |
| Restricciones 300 ± 20 mm (YZ) y 320 ± 20 mm (XY) | Notion Mecánica §3 | Cumplido |

## Entrega 2 · Cinemática y MoveIt

| Requisito | Evidencia | Estado |
|---|---|---|
| Tabla DH | `resultados/estacion2/entrega2.md` §1 | Cumplido |
| Directa propia verificada en MATLAB | `cinematica.py`, `matlab/station2/`, `matlab_prueba_cinematica.txt` | Cumplido |
| Inversa propia verificada contra la directa | `entrega2.md`, `prueba_cinematica` | Cumplido |
| Cobertura del espacio de trabajo | `entrega2.md`, `prueba_cinematica` | Cumplido |
| Paquete MoveIt: SRDF y grupo | `src/station2_moveit_config/` | Cumplido |
| Punto a punto con código propio y con MoveIt | `comparacion_moveit_vs_propia.md` | Cumplido; falta el video |
| Integración con ros2_control | `station2.ros2_control.xacro`, controladores activos | Cumplido |
| Arquitectura de software, nodos, temas y servicios | `entrega3.md` §3 | Cumplido |
| Diagrama de flujo del software | — | Por revisar: no lo encontré como figura |
| Nodo de control de trayectoria | `punto_a_punto.py`, `trayectoria.py` | Cumplido |

## Entrega 3 · Simulación integrada

| Requisito | Evidencia | Estado |
|---|---|---|
| Tres estaciones en el mismo Gazebo | `prueba_linea_completa.md` | Cumplido |
| Comunicación por condición de inicio y fin | contrato `/line/station2/start`, `/done`, `/state` | Cumplido |
| Ejecución completa de la tarea | `prueba_linea_orquestador.md`, `prueba_linea_completa.md` | Cumplido |
| Video de la línea completa | — | Pendiente |
| Estación 2: precisión < 20 mm | `prueba_estacion.md`: 3/3, error 0.000 mm | Cumplido |
| Estación 2: trayectoria condicional según la clasificación | Hoy la categoría va en el evento, pero las tres piezas se dejan en el mismo punto | Por decidir (ver abajo) |

## Lo que falta para cerrar

1. **Trayectoria condicional.** La guía pide que la trayectoria dependa de la
   clasificación. Hoy solo cambia la categoría del evento. La idea de Paola,
   con tres posiciones de entrega, lo resuelve, pero hay que acordarla con la
   Estación 3, que hoy recoge en un único `handoff_2_3`.
2. **Agarre en la primera recogida**: corregido (la pinza cierra según el giro del cubo respecto a ella; 3/3, 0.000 mm).
3. **Videos** de la Entrega 2 (punto a punto propio y con MoveIt) y de la
   Entrega 3 (línea completa).
4. **Notion** en el espacio de Santiago. La conexión de este equipo solo ve la
   copia de Diego. El contenido de la Entrega 1 ya está listo en
   `docs/estacion2/notion/`; faltan el de las Entregas 2 y 3.
5. **Push** de `estacion2-cad` y `estacion2-ros`, y PR a `main` si el equipo
   lo pide.
6. Diagrama de flujo del software, si la guía lo pide como figura.
