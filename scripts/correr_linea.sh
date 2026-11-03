#!/usr/bin/env bash
# Abre la linea completa (3 estaciones reales + orquestador).
# Uso, desde la raiz del repo y despues de colcon build:
#      scripts/correr_linea.sh [n_parts]       (por defecto 30: 10 por caja en
#                                               las 3 cajas de la E3)
#      MODO=fondo scripts/correr_linea.sh 3    -> todo en esta terminal, logs en ~/linea_logs
#
# Por defecto abre UNA ventana de Terminator con 4 paneles (E1, E2, E3,
# Orquestador). Si no hay Terminator, usa gnome-terminal (4 ventanas).
# El espacio de trabajo es la carpeta del repo, donde este el script.
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DISTRO=${ROS_DISTRO:-jazzy}
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "No encuentro $WS/install/setup.bash: compila antes con colcon build en $WS"; exit 1
fi
N=${1:-30}
N=${N%.}                      # "3." -> 3
if ! [[ $N =~ ^[0-9]+$ ]]; then
  echo "n_parts debe ser un entero (recibi '$1')"; exit 1
fi

# Las terminales graficas necesitan el bus de sesion y la pantalla.
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}
export DISPLAY=${DISPLAY:-:0}
export DBUS_SESSION_BUS_ADDRESS=${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}

# Cierra cualquier simulacion anterior y ESPERA a que muera del todo: si un
# Gazebo viejo sigue cerrandose cuando arranca el nuevo, los controladores
# de la E1 no cargan y la linea no se mueve.
PATRON="gz sim|ros2 launch|$WS/install/|robot_state_publisher|ros_gz_bridge/parameter_bridge|linea_terminator[.]conf"
if pgrep -f "$PATRON" >/dev/null; then
  echo "Cerrando la simulacion anterior..."
  pkill -INT -f "ros2 launch" 2>/dev/null
  pkill -f "linea_terminator[.]conf" 2>/dev/null   # ventanas de la corrida anterior
  for i in $(seq 1 20); do pgrep -f "$PATRON" >/dev/null || break; sleep 1; done
  pkill -9 -f "$PATRON" 2>/dev/null
  sleep 2
fi

ENV="source /opt/ros/$DISTRO/setup.bash; source $WS/install/setup.bash; \
export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:$WS/install/station1_description/share:$WS/install/station2_description/share:$WS/install/station3_description/share; \
cd $WS"

# Titulo | espera antes de arrancar (s) | nombre del log | comando
PASOS=(
  "E1 + Gazebo|0|e1|ros2 launch station1_description simulacion.launch.py"
  "Estacion 2|15|e2|ros2 launch station2_description simulacion_en_linea.launch.py"
  "Estacion 3|25|e3|ros2 launch station3_description simulacion_en_linea.launch.py"
  "Orquestador|35|orquestador|ros2 launch line_bringup line.launch.py mock_stations:=' ' n_parts:=$N"
)

TERM_CMD=""
if [ "${MODO:-}" != fondo ]; then
  if command -v terminator >/dev/null; then TERM_CMD=terminator
  elif command -v gnome-terminal >/dev/null; then TERM_CMD=gnome
  fi
fi

case $TERM_CMD in
terminator)
  # Configuracion propia (no toca ~/.config/terminator/config): una ventana
  # con 4 paneles; cada panel espera su turno antes de lanzar.
  CONF=$HOME/.config/linea_terminator.conf
  {
    echo "[global_config]"
    echo "[keybindings]"
    echo "[profiles]"
    echo "  [[default]]"
    echo "[layouts]"
    echo "  [[linea]]"
    echo "    [[[ventana]]]"
    echo "      type = Window"
    echo "      parent = \"\""
    echo "    [[[v]]]"
    echo "      type = VPaned"
    echo "      parent = ventana"
    echo "    [[[arriba]]]"
    echo "      type = HPaned"
    echo "      parent = v"
    echo "      order = 0"
    echo "    [[[abajo]]]"
    echo "      type = HPaned"
    echo "      parent = v"
    echo "      order = 1"
    padres=(arriba arriba abajo abajo)
    for i in 0 1 2 3; do
      IFS='|' read -r titulo espera log cmd <<<"${PASOS[$i]}"
      echo "    [[[panel$i]]]"
      echo "      type = Terminal"
      echo "      parent = ${padres[$i]}"
      echo "      order = $((i % 2))"
      echo "      title = $titulo"
      echo "      command = bash -c \"sleep $espera; $ENV; $cmd; exec bash\""
    done
    echo "[plugins]"
  } > "$CONF"
  # --no-dbus: si no, un Terminator ya abierto reusa la configuracion vieja.
  terminator --no-dbus -g "$CONF" -l linea -T "Linea SCARA" >/dev/null 2>&1 &
  echo "Terminator abierto: la linea arranca en unos 35 s."
  ;;
gnome)
  for paso in "${PASOS[@]}"; do
    IFS='|' read -r titulo espera log cmd <<<"$paso"
    gnome-terminal --title="$titulo" -- bash -c "sleep $espera; $ENV; $cmd; exec bash"
  done
  echo "gnome-terminal abierto: la linea arranca en unos 35 s."
  ;;
*)
  LOGS=$HOME/linea_logs
  mkdir -p "$LOGS"
  PIDS=()
  echo "Sin terminal grafica: todo corre aqui, logs en $LOGS. Ctrl+C cierra todo."
  trap 'echo; echo "Cerrando..."; kill -INT "${PIDS[@]}" 2>/dev/null; sleep 3; pkill -f "gz sim"; exit 0' INT TERM
  for paso in "${PASOS[@]}"; do
    IFS='|' read -r titulo espera log cmd <<<"$paso"
    bash -c "sleep $espera; $ENV; exec $cmd" > "$LOGS/$log.log" 2>&1 &
    PIDS+=($!); echo "  $titulo -> $LOGS/$log.log"
  done
  sleep 36
  tail -f "$LOGS/orquestador.log" & PIDS+=($!)
  wait
  ;;
esac
