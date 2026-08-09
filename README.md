# Proyecto R — Línea Simulada

Línea simulada de clasificación y empaque de componentes electrónicos.
Tres estaciones robóticas de 3 GDL, una por estudiante, desarrolladas en ROS2 y Gazebo.

Universidad EIA · Ingeniería Mecatrónica · Robótica y Control Digital

La documentación técnica del proyecto vive en Notion. Este repositorio contiene el
código, los modelos y los archivos de configuración.

## Estaciones

| Estación | Responsable | Configuración | Restricción |
|---|---|---|---|
| 1 — Alimentación y singulación | Santiago Fernando Machado Sanchez | RRP | Tiempo de ciclo de 30 s |
| 2 — Clasificación e inspección | Diego Oxman Sabogal | RPR | Precisión de 20 mm |
| 3 — Empaque | Juan David Guerra Cabrera | PRR | 10 unidades por caja |

## Entorno

Definido en ADR-001. Debe ser idéntico en las tres máquinas.

- Ubuntu 24.04 LTS
- ROS2 Jazzy Jalisco
- Gazebo Harmonic (`gz sim` 8.x)
- `gz_ros2_control` y `ros_gz_bridge`

Instalación desde cero:

```bash
bash setup/install.sh
```

## Compilar y ejecutar

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Ejecutar la línea completa. Mientras no existan los controladores, las tres
estaciones corren como nodos de prueba:

```bash
ros2 launch line_bringup line.launch.py
```

A medida que una estación tenga su controlador, se retira de la lista:

```bash
ros2 launch line_bringup line.launch.py mock_stations:=station2,station3
```

Número de piezas de la corrida:

```bash
ros2 launch line_bringup line.launch.py n_parts:=10
```

## Estructura

```
├── setup/install.sh          Instalación del entorno
├── docs/datasheets/          Hojas de datos de los componentes comerciales
├── cad/stationN/             Modelos CAD, planos y STL exportados
├── matlab/                   Verificación numérica de la cinemática
└── src/
    ├── line_interfaces/      Mensajes y servicios comunes
    ├── line_bringup/         Mundo, poses de la línea y lanzamiento
    ├── line_orchestrator/    Orquestador y nodo de prueba de estación
    └── stationN_*/           Descripción y control de cada estación
```

Los paquetes de estación se crean a medida que se necesitan:

```bash
cd src
ros2 pkg create --build-type ament_cmake station1_description
ros2 pkg create --build-type ament_python station1_control
```

## Protocolo entre estaciones

Cada estación expone la misma interfaz y no se suscribe a los eventos de ninguna
otra. La secuencia la gobierna el orquestador. Para `N` igual a 1, 2 o 3:

| Endpoint | Tipo | Dirección |
|---|---|---|
| `/line/stationN/start` | Servicio `StartTask` | Entrada |
| `/line/stationN/done` | Tópico `HandoffEvent` | Salida, una vez por ciclo |
| `/line/stationN/state` | Tópico `StationState` | Salida, continuo a 2 Hz |

Reglas: una estación ocupada responde `accepted=false` y no encola peticiones;
al terminar publica exactamente un evento de fin y vuelve a `IDLE`; ante un fallo
publica `state=ERROR` y no publica en `done`.

La especificación completa está en Notion, en Programación.

## Archivos de propiedad compartida

Tres archivos afectan a los tres integrantes. Toda modificación se acuerda entre
los tres, porque un cambio aquí rompe o desplaza el trabajo de otra estación.

- `src/line_interfaces/msg/*.msg` y `srv/*.srv`
- `src/line_bringup/config/line_poses.yaml`
- `src/line_bringup/worlds/line.sdf`

## Convenciones

Ramas: `main` se mantiene siempre en un estado que compila. Una rama por tarea,
con el prefijo del ámbito. Toda integración pasa por pull request.

Mensajes de commit con prefijo de ámbito:

```
[E1]   Estación 1
[E2]   Estación 2
[E3]   Estación 3
[INT]  Integración: mundo, orquestador, line_poses.yaml
[DOC]  Documentación y referencias
[ENV]  Entorno: install.sh, dependencias
```

Cada integrante commitea con su propia cuenta.

Los modelos CAD y los STL se versionan por ser entregables. Los videos y renders
se publican en Notion, no aquí.

## Sistemas de referencia

Frame raíz único `world`. Cada estación opera bajo su namespace `/stationN` y
prefija todos sus frames con `stationN/`. Unidades del Sistema Internacional:
metros, kilogramos, radianes y segundos.

Ninguna estación define poses de traspaso en su código: todas las leen de
`src/line_bringup/config/line_poses.yaml`.

## Decisiones pendientes

- **Git LFS.** Sin configurar. Si se van a versionar los archivos nativos de CAD
  (SLDPRT, F3D), conviene activarlo antes de añadirlos; migrar después es costoso.
  Si solo se versionan STL decimados y planos en PDF, no hace falta.
- **Verificación de cinemática.** La carpeta `matlab/` asume MATLAB, que es lo que
  nombra la guía. Si se prefiere Python con SymPy, basta con renombrarla.
