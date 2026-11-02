#!/bin/bash
# Compila y prueba JAR en la imagen Docker de ROS Humble, sin tocar build/ ni install/.
#
#   tools/verificar.sh tests        colcon build + colcon test
#   tools/verificar.sh bag [N]      pipeline del robot real sobre el bag, N corridas (1)
#   tools/verificar.sh              las dos cosas, 1 corrida
#
# Cada corrida es un contenedor propio con su ROS_DOMAIN_ID: al terminar se cierra
# entero, y no recibe ni publica tópicos de otros nodos de ROS que estén corriendo.
# Variables: IMAGEN (yahboom_rosmaster:lucho), WS (~/rosmaster_ws).
# Requiere jar_workshops en $WS/src (evaluador_victimas) y el bag en $WS/bags.
set -euo pipefail
IMAGEN=${IMAGEN:-yahboom_rosmaster:lucho}
WS=${WS:-$HOME/rosmaster_ws}

contenedor() {
    # $1: dominio de ROS; el resto, argumentos para la parte de adentro.
    local dominio=$1
    shift
    docker run --rm -e ROS_DOMAIN_ID="$dominio" \
        -v "$WS/src:/src:ro" -v "$WS/bags:/bags:ro" \
        -v "$(realpath "$0"):/verificar.sh:ro" \
        --entrypoint bash "$IMAGEN" /verificar.sh dentro "$@"
}

if [[ "${1:-}" != dentro ]]; then
    MODO=${1:-todo}
    CORRIDAS=${2:-1}
    [[ -d "$WS/src/jar_workshops" ]] || { echo "Falta $WS/src/jar_workshops"; exit 1; }
    if [[ "$MODO" != tests && ! -d "$WS/bags/laberinto_victimas" ]]; then
        echo "Falta el bag en $WS/bags/laberinto_victimas (ver README de la semana 10)"
        exit 1
    fi
    # Dominios 150-199: lejos del 0 que usan el simulador y los launch por defecto.
    base=$((150 + RANDOM % 40))
    if [[ "$MODO" == tests || "$MODO" == todo ]]; then
        contenedor "$base" tests
    fi
    if [[ "$MODO" == bag || "$MODO" == todo ]]; then
        [[ "$MODO" == todo ]] && CORRIDAS=1
        for i in $(seq 1 "$CORRIDAS"); do
            echo "== corrida $i/$CORRIDAS: bag -> robot_real.launch.py -> /victimas -> evaluador"
            contenedor $((base + i)) bag
        done
    fi
    exit 0
fi

# ---------- dentro del contenedor ----------
set +eu  # los setup.bash de ROS usan variables sin definir
source /opt/ros/humble/setup.bash
mkdir -p /tmp/ws && cd /tmp/ws
colcon build --base-paths /src --build-base build --install-base install \
    --packages-up-to jar_bringup jar_navegacion evaluador_victimas \
    --event-handlers console_cohesion- status- > build.log 2>&1 \
    || { echo "colcon build falló:"; tail -20 build.log; exit 1; }
source install/setup.bash

if [[ "$2" == tests ]]; then
    echo "== colcon build"
    grep -E "Summary" build.log
    echo "== colcon test"
    colcon test --base-paths /src --build-base build --install-base install \
        --test-result-base resultados --event-handlers console_cohesion- status- \
        --packages-select jar_bringup jar_localizacion jar_percepcion jar_mision \
        jar_navegacion jar_exploracion > /dev/null 2>&1
    colcon test-result --test-result-base resultados | tail -1
    exit 0
fi

# Corrida del bag: los nodos mueren junto con el contenedor al salir.
ros2 launch jar_bringup robot_real.launch.py rviz:=false > pipeline.log 2>&1 &
ros2 launch evaluador_victimas evaluador.launch.py > evaluador.log 2>&1 &
sleep 8
ros2 bag play /bags/laberinto_victimas --clock > /dev/null 2>&1
sleep 3
grep -E "Víctima" pipeline.log | sed 's/^.*\] \[/[/'
grep -E "Puntaje|caja roja" evaluador.log | tail -3 \
    | sed 's/^\[evaluador-1\] //; s/^\[INFO\] \[[^]]*\] //'
