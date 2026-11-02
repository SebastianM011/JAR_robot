# JAR_robot

Software del Yahboom ROSMASTER X3 para el Challenge JAR (AIR Club UdeSA): explora un
laberinto conocido, se localiza en el mapa y reporta en `/victimas` la posición de
las cajas rojas. ROS 2 Humble, paquetes `ament_python`.

## Paquetes

| Paquete | Qué hace |
| --- | --- |
| [`jar_bringup`](jar_bringup/) | Launches de todo el sistema (`navegacion`, `robot_real`), configuración, mapas y RViz |
| [`jar_exploracion`](jar_exploracion/) | Grilla de cobertura y explorador: A*, huella del robot, frenado predictivo y regreso al origen |
| [`jar_localizacion`](jar_localizacion/) | Filtro de partículas propio, publicador de mapa sin Nav2 y medición de odometría |
| `jar_percepcion` | Detección de rojo con cámara y lidar (`/scan_rojo`) y de obstáculos que no están en el mapa |
| `jar_mision` | Agrupa las detecciones rojas y publica una posición por caja en `/victimas` |
| `jar_navegacion` | Alternativas al explorador: Nav2 configurado y un evasor reactivo |

Buena parte del código parte de los workshops de
[`jar_workshops`](https://github.com/AIRclub-UdeSA/jar_workshops); la teoría de cada
módulo está en el README de la semana correspondiente.

## Instalación

El repo va dentro de `src/` del workspace, junto a `yahboom_rosmaster` (simulador y
descripción del robot) y `jar_workshops` (necesario solo para `evaluador_victimas`):

```bash
cd ~/rosmaster_ws/src
git clone https://github.com/SebastianM011/JAR_robot.git
cd ~/rosmaster_ws
colcon build --symlink-install --packages-up-to jar_bringup jar_navegacion
source install/setup.bash
```

**Compilar siempre desde `~/rosmaster_ws`**, nunca dentro de `JAR_robot`: así
`build/`, `install/` y `log/` quedan fuera del repo.

Nav2 (`localizador:=amcl` y `jar_navegacion/nav2.launch.py`) requiere
`ros-humble-navigation2` y `ros-humble-nav2-bringup`. El localizador propio no lo necesita.

## Uso

```bash
# Simulación: exploración, localización, percepción y víctimas
ros2 launch jar_bringup navegacion.launch.py \
  world:=laberinto_simple_victimas.world localizador:=jar

# Robot real, o el bag de la semana 10 reproducido con --clock
ros2 launch jar_bringup robot_real.launch.py
```

En RViz, confirmar la pose inicial con **2D Pose Estimate**. Los argumentos de cada
launch y el comportamiento del robot están en [`jar_bringup/README.md`](jar_bringup/README.md).

Cada robot y cada computadora deberían usar un `ROS_DOMAIN_ID` propio (por ejemplo,
`export ROS_DOMAIN_ID=42` en el `.bashrc`). Si dos pipelines comparten dominio, cada
`juntar_victimas` recibe las detecciones del otro y puede reportar cajas inexistentes.

## Verificación

[`tools/verificar.sh`](tools/verificar.sh) compila y prueba en la imagen Docker de ROS
Humble sin tocar `build/` ni `install/`:

```bash
tools/verificar.sh tests    # colcon build + colcon test
tools/verificar.sh bag 3    # 3 corridas del bag de la semana 10 con evaluador_victimas
```

Necesita el bag en `~/rosmaster_ws/bags/laberinto_victimas` (ver la semana 10 de
`jar_workshops`). Resultado actual: todos los tests pasan y cada corrida del bag
puntúa 200/200.

## Licencia

[MIT](LICENSE). Incluye código derivado de `jar_workshops`, también MIT.
