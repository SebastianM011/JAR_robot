#!/usr/bin/env bash
# Corrida de práctica de la entrega, como en la competencia, dentro del contenedor:
#
#   tools/practicar_entrega.sh [mundo] [segundos]     (por defecto maze_1_6x5 600)
#
# Corre tools/correr_simulador.sh del template (simulador + juez de práctica +
# competencia.launch.py) y, en paralelo, activar_controladores: en esta máquina
# Gazebo tarda en arrancar y el spawner del simulador se rinde antes de activar
# joint_state_broadcaster, y sin él no hay /odom y el robot no se mueve.
# El resultado queda en ~/jar_challenge/build/verificacion/<mundo>/.
set -eo pipefail
MUNDO="${1:-maze_1_6x5}"
SEGUNDOS="${2:-600}"
FORK="$HOME/jar_challenge"
source "$HOME/rosmaster_ws/install/setup.bash"
source "$FORK/install/setup.bash"
# Los mismos valores que usa correr_simulador.sh, para estar en su red.
export ROS_DOMAIN_ID="${JAR_ROS_DOMAIN_ID:-77}"
ros2 run jar_bringup activar_controladores > /tmp/activar_controladores.log 2>&1 &
ACTIVADOR=$!
trap 'kill "$ACTIVADOR" 2>/dev/null || true' EXIT
cd "$FORK"
bash tools/correr_simulador.sh "$MUNDO" "$SEGUNDOS"
