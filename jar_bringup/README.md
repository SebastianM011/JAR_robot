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
AMCL, `grilla_cobertura` y `explorador`. Usa `laberinto_simple.world` y su mapa.
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

Los paquetes `jar_localizacion`, `jar_navegacion`, `jar_percepcion` y
`jar_mision` todavía no tienen ejecutables. Sus futuros launch se pueden
incluir desde este archivo; al integrar el localizador propio, desactivar o
reemplazar el bloque AMCL. No se usan transformaciones estáticas ficticias
para suplir localización.

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
