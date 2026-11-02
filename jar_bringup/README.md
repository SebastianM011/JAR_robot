# Inicio de navegación

Desde la raíz del workspace (`~/jar_ws`), compilar también los paquetes del
simulador que están en `src/yahboom_rosmaster`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to jar_bringup
source install/setup.bash
ros2 launch jar_bringup navegacion.launch.py
```

El launch inicia Gazebo mediante el bringup de Yahboom, RViz con configuración propia, `map_server`,
AMCL (o el filtro propio), `grilla_cobertura`, `explorador`, percepción y misión.
Publica las víctimas confirmadas en `/victimas` sin abrir otro launch. Usa `laberinto_simple.world` y su mapa.
AMCL usa el modelo omnidireccional del X3. No se inicia el controlador Nav2:
el planificador y el movimiento son los implementados en `jar_exploracion`.

RViz abre `config/navegacion.rviz`, con mapa, cobertura, camino, objetivo y
la herramienta de pose inicial configurados. Para usar una configuración guardada,
pasar `rviz_config:=/ruta/archivo.rviz`.

El nodo `visualizador_grillas` convierte `/map` y `/grilla_cobertura` a marcadores
en `/mapa_visual` y `/cobertura_visual`. La configuración utiliza displays
**Marker** para evitar el shader de texturas del display **Map**, que puede fallar
con `active samplers with a different type`. Si se carga una configuración vieja,
desactivar sus displays Map y usar esos dos tópicos Marker con durabilidad
**Transient Local**. Los tópicos OccupancyGrid originales siguen disponibles.

Confirmar la posición y orientación real del robot con **2D Pose Estimate**.
Después de 3 segundos, con mapa, TF y scan disponibles, comienza la exploración.
La cobertura pasa de rojo a verde solo cuando el láser realmente observa la zona.
Si una zona no admite entrar con el margen del robot, el explorador busca un punto
alcanzable desde donde observar sus celdas libres sin atravesar paredes. Las
posiciones sin avance se descartan temporalmente y se prueban otras vistas.
`sin_destinos_transitables` indica pendientes sin más destinos o vistas encontrados;
no equivale a `terminado` ni garantiza que las zonas sean físicamente invisibles.

`Ctrl+C` cierra el launch completo. Para iniciar sin movimiento autónomo:

```bash
ros2 launch jar_bringup navegacion.launch.py exploracion:=false
```

Para usar otro escenario, pasar juntos el mundo y su mapa correspondiente:

```bash
ros2 launch jar_bringup navegacion.launch.py \
  world:=cafe.world \
  map:="$(ros2 pkg prefix --share yahboom_rosmaster_gazebo)/maps/cafe.yaml"
```

También se pueden ajustar `spawn_x`, `spawn_y`, `spawn_yaw`, `gui`, `rviz`,
`params_file` y `cmd_vel_topic`. La configuración de los nodos está en
`config/navegacion.yaml`.

Si Gazebo ya está corriendo, agregar `simulacion:=false use_sim_time:=true`.
Con `simulacion:=false`, `gui` no tiene efecto; RViz sigue controlado por `rviz`.
Si ya existe un proveedor de mapa y localización, agregar `localizacion:=false`
para evitar duplicar `/map` y la transformación `map -> odom`.
Para hardware usar `simulacion:=false use_sim_time:=false map:=/ruta/mapa.yaml`;
los drivers deben proveer `/scan`, odometría y TF `odom -> base_footprint`,
y consumir `/cmd_vel`. Este launch no inicia drivers del robot físico.

Para usar el filtro de partículas propio en lugar de AMCL:

```bash
ros2 launch jar_bringup navegacion.launch.py localizador:=jar
```

Con `localizador:=jar` se inician `publicar_mapa`, `campo_verosimilitud` y
`localizador` de `jar_localizacion` (sus parámetros están en
`config/navegacion.yaml`). Ese modo no usa Nav2: la imagen Docker
`yahboom_rosmaster` no trae `nav2_map_server` ni `nav2_amcl`, así que ahí solo
funciona `localizador:=jar`. **2D Pose Estimate** re-siembra las partículas.
No se usan transformaciones estáticas ficticias para suplir localización.

Los demás paquetes tienen sus propios launch:

- `ros2 launch jar_localizacion localizacion.launch.py`: `map_server` y filtro propio.
- `ros2 launch jar_percepcion percepcion.launch.py`: `detector_color`
  (`/rojo_detectado`), `detector_scan` (`/scan_rojo`) y `detector_obstaculos`
  (`/objetos_no_mapeados`). Requiere `/likelihood_map` y `map -> odom`.
- `ros2 launch jar_mision victimas.launch.py`: `/scan_rojo` → `/objetos_rojos`
  → `juntar_victimas` → `/victimas`. Requiere localización.
- `ros2 launch jar_navegacion nav2.launch.py`: Nav2 completo (semana 09); con
  `localizacion:=false` no inicia AMCL ni `map_server`.
- `ros2 launch jar_navegacion evasion.launch.py`: simulador y evasor reactivo
  (semana 03). En vez del cono del workshop, comprueba el recorrido de la misma
  huella de 0.37 × 0.24 m que usa el explorador (`jar_exploracion.seguridad`) y
  frena si el scan no trae datos. Publica en `cmd_vel`; no combinar con el
  explorador ni con Nav2.
- `ros2 run jar_localizacion medir_odometria` y `odom_corregida`: miden y
  muestran el error de escala de `/odom` contra IMU y lidar (semana 10).
  `odom_corregida` publica en los mismos caminos que `localizador`; no
  correrlos juntos.

## Robot real y bag de la semana 10

`robot_real.launch.py` localiza al robot en `maps/laberinto_vacio.yaml` con el
filtro propio y reporta las cajas rojas en `/victimas`. No inicia drivers ni
requiere Nav2.
Para reproducir el bag de la semana 10:

```bash
# Terminal 1
ros2 bag play ~/rosmaster_ws/bags/laberinto_victimas --clock -p
# Terminal 2
ros2 launch jar_bringup robot_real.launch.py
```

`config/robot_real.yaml` tiene las escalas medidas en ese bag:
`escala_avance: 0.784` (lidar trasero contra `/odom`) y `escala_giro: 0.804`
(IMU contra `/odom`). El IMU del bag tiene el giro con el signo invertido; el
scan matching del lidar coincide en signo con `/odom`, así que la escala es
positiva. Sin escalas el filtro se pierde después de los primeros giros.
`giro_maximo: 0.3` evita marcar rojo mientras el robot gira rápido.

Corrido en ROS Humble (imagen `yahboom_rosmaster`) sobre el bag completo, con
`evaluador_victimas` de `jar_workshops`, puntúa **200/200** en cada corrida: las
cajas quedan a ~1 cm y ~3 cm de sus centros reales. Para comprobarlo:

```bash
tools/verificar.sh tests    # colcon build + colcon test
tools/verificar.sh bag 3    # 3 corridas del bag con el evaluador (~8 min)
```

El script corre cada prueba en un contenedor propio con un `ROS_DOMAIN_ID` aparte.
Si dos pipelines comparten dominio, `juntar_victimas` recibe detecciones de los
dos y puede reportar cajas inexistentes: no correr esta prueba con otro
`robot_real.launch.py` abierto en el mismo dominio.

## Renderizado de RViz

RViz usa `QT_QPA_PLATFORM=xcb`. El renderizado por CPU es opcional
(`rviz_software:=true`); por defecto se usa aceleración gráfica. La configuración
con marcadores evita el shader de mapas sin depender de esa opción. Las variables
de renderizado se aplican solo al proceso RViz.

## Robot trabado y deslizamiento en simulación

La odometría principal integra las ruedas. Si patinan contra una pared, puede
indicar desplazamiento aunque el chasis no avance. AMCL estima la corrección
`map -> odom` usando el láser; la estimación puede divergir y la visualización
por sí sola no demuestra movimiento real.

Con `simulacion:=true`, `verificar_movimiento_simulado` se activa por defecto.
El explorador compara desplazamientos relativos de `/ground_truth/odom` y de
la pose estimada durante órdenes de traslación sostenidas. Si en 3 segundos
el desplazamiento real es menor a 2 cm y el estimado supera 8 cm, frena y pasa
a `bloqueado_deslizamiento`. Esta medición se usa solo como supervisión de la
simulación; no publica TF ni sustituye la localización. La comparación descarta
ventanas con pausas, retrocesos del reloj o huecos de datos mayores a 0.5 s.

Antes de reanudar, resolver el atasco y corregir la pose con **2D Pose Estimate**.
Si ya se acumuló cobertura con una pose incorrecta, reiniciar la prueba para
no conservar esas marcas. Para hardware el supervisor está desactivado por
defecto; al usar un simulador externo se puede activar con
`verificar_movimiento_simulado:=true`.

El seguimiento solo salta waypoints cuando el segmento hacia el siguiente
permanece en el mapa inflado y no corta esquinas de obstáculos. La protección
de deslizamiento no corrige por sí sola errores de AMCL, del mapa o del chasis.

## Pasillos estrechos y control revisado

La planificación usa ahora una huella rectangular orientada de 0.37 × 0.24 m,
que envuelve chasis y ruedas, con 0.05 m de margen. Sustituye `radio_robot`.
`subdivisiones_mapa=2` permite centrar mejor el robot entre celdas del mapa
original sin adelgazar los obstáculos.
`ganancia_repulsiva` y `radio_repulsion` ya no se utilizan: se eliminó la suma de
fuerzas de los rayos. El comando, incluido el suavizado, pasa por comprobación
predictiva de huella, mapa y scan. La velocidad cerca de paredes queda limitada
por `velocidad_pasillo` (0.10 m/s); `velocidad_maxima` sigue en 0.30 m/s.

La geometría tiene en cuenta la orientación actual. No ordena giros autónomos para
entrar de frente en pasos que solo admiten otra orientación. No se fuerza el paso
si no cabe con el margen o si el láser detecta un obstáculo.

La revisión y las pruebas reproducibles se describen en
`../jar_exploracion/docs/auditoria_exploracion.md`. Falta validar en Gazebo el paso
concreto que originó el atasco; las pruebas cinemáticas no simulan sus contactos.

## Finalización por porcentaje

`porcentaje_objetivo` es el porcentaje de celdas libres de la grilla de cobertura
que debe observar el láser para finalizar la misión. Paredes y desconocidos no
entran en el denominador. Por defecto es **98%**. Al alcanzarlo, pasa a `regresando_al_origen` en `/estado` y planifica
el regreso con A* y las mismas comprobaciones de mapa, huella y láser.
El origen se guarda en coordenadas del mapa al iniciar la exploración, después
del asentamiento de la primera pose confirmada; las correcciones posteriores
de **2D Pose Estimate** conservan ese origen. Al quedar a 10 cm o menos del
origen publica `terminado`, elimina camino y destino y publica velocidad cero.
Si el regreso está bloqueado, frena y reintenta sin declarar éxito. El origen
se conserva durante la ejecución del nodo; al reiniciarlo se guarda uno nuevo. `100` exige cobertura completa.

```bash
ros2 launch jar_bringup navegacion.launch.py porcentaje_objetivo:=90
```

Se combina con `world` y `map` para cualquier escenario. Consultar el valor actual
con `ros2 topic echo /porcentaje_cobertura --once`. El umbral admite valores
mayores que 0 y hasta 100. Los estados de bloqueo por deslizamiento conservan
su prioridad: no se declara éxito mientras la localización está invalidada.

## Cajas y obstáculos fuera del mapa

El explorador incorpora los impactos válidos del láser al mapa de planificación,
con la misma huella rectangular y margen de seguridad. Si bloquean el camino,
descarta la ruta anterior y vuelve a ejecutar A* para buscar un rodeo. El mapa
original y la cobertura no se modifican. La memoria de cada impacto dura ocho
segundos desde su última observación; una corrección con **2D Pose Estimate**
la descarta. El control predictivo sigue comprobando el scan antes de moverse.
Si no existe paso con el margen requerido, no fuerza el paso.

## Mirada de 360° para la cámara

La cobertura se marca con el láser, que ve 360°, pero las víctimas se confirman
con la cámara, que mira solo hacia adelante (unos 61° de campo visual), y el
explorador se traslada sin girar. Para que la cámara vea todo, al empezar a explorar
y cada `distancia_entre_miradas` (1.5 m) el robot gira 360° en el lugar en
`pasos_mirada` tramos de 45°, con una pausa de `pausa_mirada_s` en cada uno.
En las pausas la imagen sale nítida y `detector_scan` no la descarta por girar
rápido (`giro_maximo`).

Solo gira donde el círculo que barre la huella (radio 0.27 m) está libre en el
mapa y en el láser; en un pasillo angosto la mirada queda pendiente hasta un lugar
más amplio. Termina alineado con la orientación inicial, así que no invalida el
camino planificado, y el tiempo girando no cuenta para `timeout_sin_avance_s`.
`distancia_entre_miradas: 0` lo desactiva.

## Reporte de víctimas al final

`juntar_victimas` (`modo_reporte: al_final`) no reporta mientras explora: cuando
`/estado` pasa a `regresando_al_origen` publica en `/victimas` los grupos con más
detecciones. Un falso positivo temprano ya no gasta uno de los reportes; en el bag
de la semana 10 las cajas reales juntan 280 a 434 detecciones y el falso, 1.
Si sobran reportes, también usa grupos con al menos `detecciones_minimas_final`
(10): un reporte errado no resta puntos. Para forzar el reporte:
`ros2 topic pub --once /reportar_victimas std_msgs/msg/Empty`.

## Mundos con víctimas

Cambiar siempre `world` y `map` juntos, y `cantidad_victimas` según las cajas rojas
del mundo. Todos los mapas tienen libre el punto de partida (0, 0).

| Mundo (`<nombre>_victimas.world`) | Rojas | Distractores |
| --- | --- | --- |
| `laberinto_simple` | 2 | 1 |
| `maze_1_6x5` | 2 | 2 |
| `maze_4_metal_6x6` | 2 | 2 |
| `maze_2_6x5` | 3 | 2 |
| `maze_3_6x6` | 3 | 2 |
| `laberinto_1` | 3 | 1 |

```bash
M=maze_1_6x5
ros2 launch jar_bringup navegacion.launch.py localizador:=jar cantidad_victimas:=2 \
  world:=${M}_victimas.world \
  map:=$(ros2 pkg prefix --share yahboom_rosmaster_gazebo)/maps/${M}.yaml
```

## Inicio integrado de percepción y misión

`navegacion.launch.py` inicia por defecto `jar_percepcion` y `jar_mision`, además
de simulación, localización, exploración y visualización. Se comparte un solo
`detector_scan_color` y un solo `campo_verosimilitud`, también con AMCL.
Para desactivar módulos usar `percepcion:=false` o `victimas:=false`.
Si solo se desactiva percepción, misión inicia su propio detector de scan rojo.
No iniciar además `percepcion.launch.py` o `victimas.launch.py` en otra terminal.

```bash
ros2 launch jar_bringup navegacion.launch.py \
  world:=laberinto_simple_victimas.world localizador:=jar
ros2 topic echo /victimas
```

`jar_navegacion` ofrece Nav2 y evasión como alternativas al explorador; no se
inician juntos porque publican órdenes de movimiento sobre el mismo robot.
Las herramientas de medición de odometría se ejecutan por separado; el nodo
`odom_corregida` comparte salidas con el localizador y tampoco se inicia aquí.
