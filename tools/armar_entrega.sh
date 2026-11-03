#!/usr/bin/env bash
# Copia los paquetes que corren en la competencia al fork de jar_challenge.
#
#   tools/armar_entrega.sh [ruta_del_fork]     (por defecto ~/jar_challenge)
#
# Solo van los paquetes del launch oficial (equipo_jar/launch/competencia.launch.py).
# Sin launch/ ni config/: en la entrega la configuración vive en
# src/equipo_jar/params/ (reglamento §9.1). equipo_jar se edita en el fork.
set -euo pipefail
ORIGEN="$(cd "$(dirname "$0")/.." && pwd)"
FORK="${1:-$HOME/jar_challenge}"
PAQUETES=(jar_localizacion jar_percepcion jar_mision jar_exploracion jar_reloj)

[[ -d "$FORK/src/equipo_jar" ]] || { echo "No encuentro $FORK/src/equipo_jar" >&2; exit 1; }
for paquete in "${PAQUETES[@]}"; do
    rm -rf "${FORK:?}/src/$paquete"
    rsync -a --exclude launch/ --exclude config/ --exclude __pycache__/ \
        --exclude .pytest_cache/ "$ORIGEN/$paquete/" "$FORK/src/$paquete/"
done
echo "Copiados ${PAQUETES[*]} a $FORK/src (commit de origen: $(git -C "$ORIGEN" rev-parse --short HEAD))"
