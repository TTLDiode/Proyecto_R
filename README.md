# Propuesta de Entrega 2 — Estación 1

Esta carpeta **no es el repositorio**. Es la propuesta de lo que entraría en él,
para revisarla antes de mover nada. Se construye aparte:

```bash
cd ~/E2_propuesta
colcon build --symlink-install       # sobre ~/proyecto-r-linea-simulada ya construido
```

## Qué hay aquí

| ruta | estado | qué es |
|---|---|---|
| `src/station1_description/` | **modificado** | copia de trabajo del paquete del repo, con la capa de `ros2_control` añadida |
| `src/station1_moveit_config/` | **nuevo** | paquete de MoveIt: SRDF, cinemática, límites, controladores |
| `src/station1_control/` | **nuevo** | cinemática propia, dinámica, trayectorias, nodo de control y nodo de estación |
| `resultados/` | — | informe de la entrega y tablas medidas |
| `evidencia/` | — | capturas y secuencia del movimiento |

### Cambios sobre `station1_description`, que es lo único que toca al repo

| archivo | cambio |
|---|---|
| `urdf/station1.ros2_control.xacro` | **nuevo**, interfaces de `ros2_control` |
| `config/station1_controllers.yaml` | **nuevo**, controladores y ganancia del lazo |
| `launch/simulacion.launch.py` | **nuevo**, Gazebo con controladores activos |
| `urdf/station1.urdf.xacro` | incluye lo anterior; **`tool0` pasa a colgar de `link_3`** |
| `urdf/parametros.xacro` | sin cambios respecto de la copia del repo |
| `CMakeLists.txt`, `package.xml` | instalan `config/`, dependencias nuevas |

El único cambio con efecto sobre la Entrega 1 es el reparentado de `tool0`, y
con la mordaza cerrada la pose es idéntica. El motivo está en
`resultados/entrega2.md`, sección 4.

## Documentos

- `resultados/entrega2.md` — el informe de la entrega, con todo lo medido
- `resultados/arquitectura-software-e1.md` — punto 4.4 de la guía
- `resultados/entrega3-infraestructura.md` — qué de la Entrega 3 ya funciona
- `resultados/comparacion_moveit_vs_propia.md` — generado por `comparar_moveit`

## Comprobación rápida

```bash
./verificar_e2.sh        # en el home de la caja
```
