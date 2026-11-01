import math
import time

import numpy as np
import scipy.ndimage as nd

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener

from jar_exploracion.planificador import a_estrella, segmento_transitable
from jar_exploracion.trazado import trazar_rayo
from jar_exploracion.monitor_movimiento import DetectorDeslizamiento

FRECUENCIA_HZ = 10.0
REPLANIFICAR_CADA_N_TICKS = int(FRECUENCIA_HZ * 2)  # cada ~2s, aunque el objetivo no haya cambiado

ESTADO_INICIALIZANDO = 'inicializando'
ESTADO_EXPLORANDO = 'explorando'
ESTADO_TERMINADO = 'terminado'
ESTADO_SIN_DESTINOS = 'sin_destinos_transitables'
ESTADO_DESLIZAMIENTO = 'bloqueado_deslizamiento'


def yaw_de_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Explorador(Node):
    """
    Máquina de estados de 3 estados que gobierna la misión completa:

        INICIALIZANDO -> EXPLORANDO -> TERMINADO

    - **INICIALIZANDO**: espera a que un humano confirme la pose inicial
      con "2D Pose Estimate" en RViz (el robot real no arranca sabiendo
      dónde está) y a que el filtro de partículas tenga un rato para
      asentarse con el robot quieto antes de arrancar a moverse —
      moverse mientras la pose todavía no convergió compone el error.
    - **EXPLORANDO**: sigue el camino hasta la celda libre no cubierta más
      cercana. Transición a TERMINADO cuando no queda ninguna (volver a
      la base, o a cualquier otro punto, queda como desafío extra — ver
      el README).
    - **TERMINADO**: se queda quieto.

    Dentro de EXPLORANDO, el movimiento en sí **no** es
    reactivo puro: en vez de ir directo al objetivo sumando un vector
    atractivo y uno repulsivo (lo que en un pasillo angosto puede quedar
    oscilando en el lugar — el mínimo local clásico de un controlador
    puramente reactivo), primero se **planifica** con A* el camino óptimo
    (semana 08 depende de tener `a_estrella()` completa en
    `planificador.py`), y después se lo sigue waypoint a waypoint. El
    vector repulsivo del /scan queda como capa de seguridad secundaria
    (obstáculos que el mapa estático no conoce), no como mecanismo
    principal de esquive.

    A* corre sobre el `/map` original (resolución fina), no sobre la
    grilla de cobertura: esa grilla agrupa varias celdas del mapa en una
    sola a resolución más gruesa para que cubrir el área entera sea
    alcanzable, y una pared delgada puede ocupar menos de la mitad de un
    bloque agrupado — quedaría marcada "libre" y A* la atravesaría sin
    darse cuenta. La grilla de cobertura solo se usa para decidir **qué
    zona** visitar (la celda libre no cubierta más cercana); una vez
    elegida esa zona, el camino hasta ahí se planifica contra el mapa de
    verdad.

    Contra ese mapa fino se aplican además dos capas, mismo mecanismo que
    un costmap inflado (tipo Nav2, o el que arma el TP Final con
    `distance_transform_edt` + campana de Gauss sobre `/likelihood_map`,
    ya resueltas acá): un **filtro duro** descarta cualquier celda a menos
    de `radio_robot + margen_seguridad` de una pared (así A* nunca elige
    un pasillo más angosto que el propio chasis), y un **costo suave** que
    decae con la distancia a la pared hace que, entre las celdas que sí
    pasan el filtro, A* prefiera las más alejadas cuando hay margen para
    elegir.
    """

    def __init__(self):
        super().__init__('explorador')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('velocidad_maxima', 0.3)
        self.declare_parameter('ganancia_atractiva', 0.6)
        self.declare_parameter('ganancia_repulsiva', 0.15)
        self.declare_parameter('radio_repulsion', 0.4)
        self.declare_parameter('suavizado', 0.3)
        self.declare_parameter('tiempo_asentamiento_s', 3.0)
        # Donatello mide 0.30 x 0.20 m (rosmaster_x3_base.urdf.xacro); el
        # radio circunscripto (mitad de la diagonal) es ~0.18 m -- usamos
        # ese valor para no ligar una esquina del chasis contra la pared
        # sin importar la orientación con la que pase (es holonómico).
        self.declare_parameter('radio_robot', 0.18)
        self.declare_parameter('margen_seguridad', 0.05)
        self.declare_parameter('sigma_costo', 0.15)
        self.declare_parameter('timeout_scan_s', 0.5)
        self.declare_parameter('reintento_objetivo_s', 10.0)
        self.declare_parameter('timeout_sin_avance_s', 15.0)
        self.declare_parameter('verificar_movimiento_simulado', False)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.velocidad_maxima = self.get_parameter('velocidad_maxima').value
        self.ganancia_atractiva = self.get_parameter('ganancia_atractiva').value
        self.ganancia_repulsiva = self.get_parameter('ganancia_repulsiva').value
        self.radio_repulsion = self.get_parameter('radio_repulsion').value
        # Peso del comando nuevo contra el anterior (media móvil exponencial),
        # para que el resultado de seguir el camino no salte bruscamente de
        # un waypoint a otro.
        self.suavizado = self.get_parameter('suavizado').value
        self.tiempo_asentamiento_s = self.get_parameter('tiempo_asentamiento_s').value
        self.radio_robot = self.get_parameter('radio_robot').value
        self.margen_seguridad = self.get_parameter('margen_seguridad').value
        self.sigma_costo = self.get_parameter('sigma_costo').value
        self.timeout_scan_s = self.get_parameter('timeout_scan_s').value
        self.reintento_objetivo_s = self.get_parameter('reintento_objetivo_s').value
        self.timeout_sin_avance_s = self.get_parameter('timeout_sin_avance_s').value
        if self.timeout_sin_avance_s <= 0:
            raise ValueError('timeout_sin_avance_s debe ser positivo')
        self.seguimiento_avance = None
        self.detector_deslizamiento = DetectorDeslizamiento()
        self.velocidad_ordenada = 0.0
        if self.timeout_scan_s <= 0 or self.reintento_objetivo_s <= 0:
            raise ValueError('Los tiempos de scan y reintento deben ser positivos')
        if not 0.0 < self.suavizado <= 1.0:
            raise ValueError('suavizado debe estar entre 0 (excluido) y 1')
        self.vx_anterior = 0.0
        self.vy_anterior = 0.0
        self.pose_inicial_confirmada = False
        self.momento_confirmacion = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.libre_fino = None      # bool[alto, ancho], resolución del /map: transitable, para A*
        self.costo_fino = None      # float[alto, ancho]: costo extra por cercanía a una pared
        self.info_mapa = None       # MapMetaData del /map (resolución fina)
        self.no_cubiertas = None    # bool[alto, ancho], resolución de grilla_cobertura
        self.info_grilla = None     # MapMetaData de grilla_cobertura (resolución gruesa)
        self.ultimo_scan = None
        self.momento_scan = None
        self.objetivos_pendientes = {}
        self.zonas_sin_destino = set()
        self.puntos_observacion = {}
        self.observaciones_fallidas = {}
        self.cursores_observacion = {}
        self.cache_busquedas = {}
        self.turnos_busqueda = {}
        self.rondas_observacion = {}
        self.cache_zonas_alcanzables = {}
        self.busqueda_pendiente = False
        self.estado = ESTADO_INICIALIZANDO

        self.objetivo_celda = None  # (fila, col) al que corresponde self.camino
        self.objetivo_mision = None  # Se conserva hasta cubrir, rechazar o suspender la zona.
        self.camino = None          # lista de (fila, col), en orden, sin recorrer
        self.ticks_desde_replan = REPLANIFICAR_CADA_N_TICKS  # fuerza plan en el primer tick

        qos_grilla = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, 'map', self.recibir_mapa, qos_grilla)
        self.create_subscription(
            OccupancyGrid, 'grilla_cobertura', self.recibir_grilla, qos_grilla)
        self.create_subscription(LaserScan, 'scan', self.recibir_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.recibir_pose_inicial, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_objetivo = self.create_publisher(Marker, 'objetivo_actual', 10)
        self.pub_camino = self.create_publisher(Path, 'camino_planificado', 10)
        self.pub_estado = self.create_publisher(String, 'estado', 10)
        if self.get_parameter('verificar_movimiento_simulado').value:
            self.create_subscription(
                Odometry, '/ground_truth/odom', self.verificar_movimiento,
                qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / FRECUENCIA_HZ, self.controlar)

    # ---------- callbacks ----------

    def recibir_mapa(self, msg: OccupancyGrid):
        alto, ancho = msg.info.height, msg.info.width
        datos = np.array(msg.data, dtype=np.int16).reshape((alto, ancho))
        self.objetivo_mision = None
        self.zonas_sin_destino.clear()
        self.puntos_observacion.clear()
        self.observaciones_fallidas.clear()
        self.cursores_observacion.clear()
        self.cache_busquedas.clear()
        self.turnos_busqueda.clear()
        self.rondas_observacion.clear()
        self.cache_zonas_alcanzables.clear()
        ocupado = datos != 0  # pared o desconocido: no confiar

        # Distancia real (en metros) de cada celda a la pared más cercana.
        dist_a_pared = nd.distance_transform_edt(~ocupado) * msg.info.resolution
        self.distancia_a_pared = dist_a_pared
        self.datos_mapa = datos

        # Filtro duro: ninguna celda a menos de radio_robot + margen de una
        # pared es "libre" para A* -- sin esto, A* podría mandar a
        # Donatello por un pasillo más angosto que su propio chasis sin
        # saber que no entra (ver README de esta semana).
        radio_total = self.radio_robot + self.margen_seguridad
        self.libre_fino = (datos == 0) & (dist_a_pared >= radio_total)
        self.componentes_libres, _ = nd.label(self.libre_fino)

        # Costo suave por encima del filtro duro: entre las celdas que sí
        # pasan, A* va a preferir las más alejadas de la pared cuando haya
        # margen para elegir -- mismo mecanismo (campana de Gauss sobre la
        # distancia a la pared) que un costmap inflado tipo Nav2.
        self.costo_fino = 100.0 * np.exp(-(dist_a_pared ** 2) / (2.0 * self.sigma_costo ** 2))

        self.info_mapa = msg.info

    def recibir_grilla(self, msg: OccupancyGrid):
        alto, ancho = msg.info.height, msg.info.width
        datos = np.array(msg.data, dtype=np.int16).reshape((alto, ancho))
        if self.info_grilla != msg.info:
            self.objetivo_mision = None
            self.zonas_sin_destino.clear()
            self.puntos_observacion.clear()
            self.observaciones_fallidas.clear()
            self.cursores_observacion.clear()
            self.cache_busquedas.clear()
            self.turnos_busqueda.clear()
            self.rondas_observacion.clear()
            self.cache_zonas_alcanzables.clear()
        self.no_cubiertas = (datos == 100)
        self.info_grilla = msg.info
        if self.objetivo_celda is not None:
            f, c = self.objetivo_celda
            if not (0 <= f < alto and 0 <= c < ancho) or not self.no_cubiertas[f, c]:
                self.get_logger().info(
                    f'Zona {self.objetivo_celda} cubierta; se libera el objetivo.')
                self.objetivo_celda = None
                self.seguimiento_avance = None
                self.camino = None
                self.publicar_camino([])

    def recibir_scan(self, msg: LaserScan):
        self.ultimo_scan = msg
        self.momento_scan = self.get_clock().now()

    def recibir_pose_inicial(self, msg: PoseWithCovarianceStamped):
        # Reinicia el asentamiento cada vez (incluso si ya se había
        # confirmado antes): si el humano corrige la pose a mitad de
        # recorrido, el robot debe volver a quedarse quieto un momento
        # para que el filtro converja de nuevo antes de seguir moviéndose.
        self.objetivos_pendientes.clear()
        self.objetivo_mision = None
        self.detector_deslizamiento.reiniciar()
        self.seguimiento_avance = None
        self.zonas_sin_destino.clear()
        self.puntos_observacion.clear()
        self.observaciones_fallidas.clear()
        self.cursores_observacion.clear()
        self.cache_busquedas.clear()
        self.turnos_busqueda.clear()
        self.rondas_observacion.clear()
        self.cache_zonas_alcanzables.clear()
        self.pose_inicial_confirmada = True
        self.momento_confirmacion = self.get_clock().now()
        if self.estado != ESTADO_INICIALIZANDO:
            self.estado = ESTADO_INICIALIZANDO
            self.camino = None
            self.get_logger().info(
                f'Transición: * -> {ESTADO_INICIALIZANDO} (pose corregida desde RViz)')

    # ---------- geometría ----------

    def verificar_movimiento(self, msg):
        """Usa la medición de Gazebo solo para detectar deslizamiento, sin publicar TF."""
        if self.estado != ESTADO_EXPLORANDO:
            self.detector_deslizamiento.reiniciar()
            return
        pose = self.obtener_pose_robot()
        if pose is None:
            self.detector_deslizamiento.reiniciar()
            return
        posicion = msg.pose.pose.position
        instante = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.detector_deslizamiento.actualizar(
                instante, (posicion.x, posicion.y), pose[:2], self.velocidad_ordenada):
            self.estado = ESTADO_DESLIZAMIENTO
            self.camino = None
            self.publicar_camino([])
            self.detener()
            self.get_logger().error(
                'Deslizamiento detectado: la pose estimada avanza, pero el chasis '
                'de Gazebo permanece quieto. Exploración detenida. Revisar el paso '
                'y corregir la pose con 2D Pose Estimate antes de continuar.')

    def obtener_pose_robot(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_de_quaternion(tf.transform.rotation)
        return x, y, yaw

    def celda_a_mundo(self, fila: int, col: int, info):
        yaw = yaw_de_quaternion(info.origin.orientation)
        c, sn = math.cos(yaw), math.sin(yaw)
        dx, dy = (col + 0.5) * info.resolution, (fila + 0.5) * info.resolution
        return (info.origin.position.x + c * dx - sn * dy,
                info.origin.position.y + sn * dx + c * dy)

    def celda_de(self, x: float, y: float, info):
        yaw = yaw_de_quaternion(info.origin.orientation)
        c, sn = math.cos(yaw), math.sin(yaw)
        dx, dy = x - info.origin.position.x, y - info.origin.position.y
        col = math.floor((c * dx + sn * dy) / info.resolution)
        fila = math.floor((-sn * dx + c * dy) / info.resolution)
        return fila, col

    def celda_fina_segura(self, x: float, y: float):
        """Devuelve None si el punto queda fuera del mapa fino."""
        fila, col = self.celda_de(x, y, self.info_mapa)
        alto, ancho = self.libre_fino.shape
        if not (0 <= fila < alto and 0 <= col < ancho):
            return None
        return fila, col

    def celda_no_cubierta_mas_cercana(self, fila_r: int, col_r: int, preferidas=None):
        """
        TODO: de todas las celdas de `self.no_cubiertas` que son `True`
        (libres y todavía no vistas), devolver la más cercana a
        `(fila_r, col_r)` -- la celda donde está el robot ahora, en la
        grilla de cobertura.

        Devolvé una tupla `(fila, col)`, o `None` si no queda ninguna
        celda no cubierta (cobertura completa).

        Pasos sugeridos:
          1. `np.where(self.no_cubiertas)` te da dos arrays paralelos,
             `filas` y `cols`, con los índices de todas las celdas
             `True`.
          2. Si `len(filas) == 0`, no queda nada por cubrir: devolvé
             `None`.
          3. Calculá la distancia (al cuadrado, no hace falta la raíz
             para comparar) de cada candidata a `(fila_r, col_r)`:
             `dist2 = (filas - fila_r)**2 + (cols - col_r)**2` --
             vectorizado, una cuenta para las N celdas a la vez.
          4. `np.argmin(dist2)` te da el índice, dentro de `filas`/`cols`,
             de la candidata más cercana.
          5. Devolvé `(int(filas[i]), int(cols[i]))` con ese índice.
        """
        candidatas = self.no_cubiertas.copy()
        for celda in self.zonas_sin_destino:
            if (0 <= celda[0] < candidatas.shape[0]
                    and 0 <= celda[1] < candidatas.shape[1]):
                candidatas[celda] = False
        ahora = self.get_clock().now().nanoseconds / 1e9
        for celda, vencimiento in list(self.objetivos_pendientes.items()):
            if ahora >= vencimiento:
                del self.objetivos_pendientes[celda]
            elif (0 <= celda[0] < candidatas.shape[0]
                  and 0 <= celda[1] < candidatas.shape[1]):
                candidatas[celda] = False
        # La cercanía elige un objetivo nuevo; no debe cambiar uno todavía válido.
        if self.objetivo_mision is not None:
            f, c = self.objetivo_mision
            if (0 <= f < candidatas.shape[0] and 0 <= c < candidatas.shape[1]
                    and candidatas[f, c]):
                return self.objetivo_mision
        self.objetivo_mision = None
        directas = candidatas & preferidas if preferidas is not None else candidatas
        if preferidas is not None:
            for f, c in self.observaciones_fallidas:
                if 0 <= f < directas.shape[0] and 0 <= c < directas.shape[1]:
                    directas[f, c] = False
        if np.any(directas):
            candidatas = directas
        elif preferidas is not None:
            # Una ronda completa antes de volver a las mismas zonas difíciles.
            filas, cols = np.where(candidatas)
            if len(filas):
                rondas = np.array([self.rondas_observacion.get((int(f), int(c)), 0)
                                   for f, c in zip(filas, cols)])
                candidatas[filas[rondas > rondas.min()], cols[rondas > rondas.min()]] = False
        filas, cols = np.where(candidatas)

        if len(filas) == 0:
            return None

        dist2 = (filas - fila_r)**2 + (cols - col_r)**2

        i = np.argmin(dist2)

        self.objetivo_mision = (int(filas[i]), int(cols[i]))
        return self.objetivo_mision

    def vector_hacia(self, x_r, y_r, yaw_r, punto_mundo, distancia_restante=None):
        """
        TODO: vector que apunta desde la pose del robot hacia
        `punto_mundo` (una tupla `(x, y)` en `map`), expresado en el
        **frame del robot** -- porque Donatello es holonómico (mecanum,
        ver semana 02): puede moverse en diagonal sin girar el chasis
        primero, así que el controlador manda `linear.x`/`linear.y`
        directo en vez de "girar hasta apuntar, avanzar" como una base
        diferencial. Para eso, el vector "hacia dónde ir" tiene que estar
        expresado relativo a hacia dónde mira el robot ahora, no en
        coordenadas de `map`.

        Devolvé una tupla `(vx, vy, dist)`: la componente x e y del
        vector en el frame del robot, y la distancia en línea recta hasta
        `punto_mundo` (esto último ya lo necesitás calcular en el camino,
        y lo usa quien te llama para saber si ya llegaste).

        Pasos sugeridos:
          1. Separá `punto_mundo` en `gx, gy`.
          2. Calculá el vector en `map`: `dx, dy = gx - x_r, gy - y_r`.
          3. Calculá `dist = math.hypot(dx, dy)`. Si es menor a `1e-3`
             (prácticamente en el punto), devolvé `(0.0, 0.0, dist)`
             directo -- dividir por una distancia ~0 en el paso siguiente
             rompería todo.
          4. Frenar a medida que se acerca, en vez de mantener velocidad
             constante hasta pegar contra el punto y frenar en seco:
             `escala = min(dist, 1.0) * self.ganancia_atractiva`, y de ahí
             `dir_x_mundo, dir_y_mundo = (dx / dist) * escala, (dy / dist) * escala`.
          5. Rotá ese vector de `map` al frame del robot usando `yaw_r`
             (mismo tipo de rotación que ya usaste transformando puntos
             del scan a `map` en semanas anteriores, pero en sentido
             inverso -- acá es de `map` al frame del robot):
               `c, s = math.cos(yaw_r), math.sin(yaw_r)`
               `vx = c * dir_x_mundo + s * dir_y_mundo`
               `vy = -s * dir_x_mundo + c * dir_y_mundo`
          6. Devolvé `(vx, vy, dist)`.
        """

        # 1. Coordenadas del objetivo
        gx, gy = punto_mundo

        # 2. Vector desde el robot hasta el objetivo
        dx = gx - x_r
        dy = gy - y_r

        # 3. Distancia al objetivo
        dist = math.hypot(dx, dy)

        if dist < 1e-3:
            return (0.0, 0.0, dist)

        # 4. Calcular velocidad proporcional a la distancia
        frenado = dist if distancia_restante is None else distancia_restante
        escala = min(frenado, 1.0) * self.ganancia_atractiva

        dir_x_mundo = (dx / dist) * escala
        dir_y_mundo = (dy / dist) * escala

        # 5. Rotar el vector al frame del robot
        c = math.cos(yaw_r)
        s = math.sin(yaw_r)

        vx = c * dir_x_mundo + s * dir_y_mundo
        vy = -s * dir_x_mundo + c * dir_y_mundo

        # 6. Devolver el vector y la distancia
        return (vx, vy, dist)

    def detener(self):
        """Frena y reinicia el filtro para no reutilizar velocidades anteriores."""
        self.vx_anterior = self.vy_anterior = 0.0
        self.velocidad_ordenada = 0.0
        self.pub_cmd_vel.publish(Twist())

    def scan_vigente(self):
        if self.ultimo_scan is None or self.momento_scan is None:
            return False
        ahora = self.get_clock().now()
        edad = (ahora - self.momento_scan).nanoseconds / 1e9
        if not 0.0 <= edad <= self.timeout_scan_s:
            return False
        stamp = self.ultimo_scan.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            edad = (ahora - Time.from_msg(stamp)).nanoseconds / 1e9
            if not 0.0 <= edad <= self.timeout_scan_s:
                return False
        return True

    def vector_repulsivo(self):
        """Capa de seguridad secundaria: empuja lejos de cualquier punto
        del /scan más cerca que radio_repulsion. El camino de A* ya evita
        las paredes que conoce la grilla; esto cubre lo que la grilla no
        sabe (resolución gruesa, obstáculos dinámicos)."""
        if not self.scan_vigente():
            return None

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ultimo_scan.header.frame_id, Time())
        except Exception:
            return None

        msg = self.ultimo_scan
        rangos = np.array(msg.ranges)
        n = len(rangos)
        angulos = msg.angle_min + np.arange(n) * msg.angle_increment
        validos = (np.isfinite(rangos) & (rangos > 0.0)
                   & (rangos >= msg.range_min) & (rangos <= msg.range_max))
        if not np.any(validos):
            return 0.0, 0.0

        r = rangos[validos]
        a = angulos[validos]
        xs_laser = r * np.cos(a)
        ys_laser = r * np.sin(a)

        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        tyaw = yaw_de_quaternion(tf.transform.rotation)
        c, s = math.cos(tyaw), math.sin(tyaw)
        xs_base = tx + c * xs_laser - s * ys_laser
        ys_base = ty + s * xs_laser + c * ys_laser

        dist = np.maximum(np.hypot(xs_base, ys_base), 1e-6)
        cerca = dist < self.radio_repulsion
        if not np.any(cerca):
            return 0.0, 0.0

        magnitud = (1.0 / dist[cerca] - 1.0 / self.radio_repulsion)
        rx = -magnitud * (xs_base[cerca] / dist[cerca])
        ry = -magnitud * (ys_base[cerca] / dist[cerca])
        return (float(np.sum(rx)) * self.ganancia_repulsiva,
                float(np.sum(ry)) * self.ganancia_repulsiva)

    # ---------- planificación ----------

    def zonas_alcanzables(self, componente):
        """Proyecta las celdas seguras conectadas al robot sobre la cobertura."""
        if componente in self.cache_zonas_alcanzables:
            return self.cache_zonas_alcanzables[componente]
        mascara = np.zeros(self.no_cubiertas.shape, dtype=bool)
        if componente == 0:
            return mascara
        filas, cols = np.where(self.componentes_libres == componente)
        mapa, grilla = self.info_mapa, self.info_grilla
        yaw = yaw_de_quaternion(mapa.origin.orientation)
        dx = (cols + 0.5) * mapa.resolution
        dy = (filas + 0.5) * mapa.resolution
        x = mapa.origin.position.x + math.cos(yaw) * dx - math.sin(yaw) * dy
        y = mapa.origin.position.y + math.sin(yaw) * dx + math.cos(yaw) * dy
        yaw = yaw_de_quaternion(grilla.origin.orientation)
        dx, dy = x - grilla.origin.position.x, y - grilla.origin.position.y
        cc = np.floor((math.cos(yaw) * dx + math.sin(yaw) * dy)
                      / grilla.resolution).astype(int)
        ff = np.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy)
                      / grilla.resolution).astype(int)
        validos = (ff >= 0) & (ff < mascara.shape[0]) & (cc >= 0) & (cc < mascara.shape[1])
        mascara[ff[validos], cc[validos]] = True
        self.cache_zonas_alcanzables[componente] = mascara
        return mascara

    def objetivo_sin_avance(self, objetivo, x, y):
        """Detecta un objetivo sin desplazamiento de al menos cinco centímetros."""
        ahora = self.get_clock().now().nanoseconds / 1e9
        anterior = self.seguimiento_avance
        if (anterior is None or anterior[0] != objetivo
                or math.hypot(x - anterior[1], y - anterior[2]) >= 0.05):
            self.seguimiento_avance = (objetivo, x, y, ahora)
            return False
        return ahora - anterior[3] >= self.timeout_sin_avance_s

    def punto_seguro_en_zona(self, zona, componente=None):
        """Busca un centro fino transitable dentro de la celda de cobertura."""
        centro = self.celda_a_mundo(*zona, self.info_grilla)
        fila, col = self.celda_de(*centro, self.info_mapa)
        # El radio cubre incluso grillas rotadas respecto del mapa fino.
        radio = math.ceil(self.info_grilla.resolution / self.info_mapa.resolution) + 1
        alto, ancho = self.libre_fino.shape
        candidatos = []
        for f in range(max(0, fila - radio), min(alto, fila + radio + 1)):
            for c in range(max(0, col - radio), min(ancho, col + radio + 1)):
                if not self.libre_fino[f, c]:
                    continue
                if componente is not None and self.componentes_libres[f, c] != componente:
                    continue
                punto = self.celda_a_mundo(f, c, self.info_mapa)
                if self.celda_de(*punto, self.info_grilla) != zona:
                    continue
                distancia = (punto[0] - centro[0])**2 + (punto[1] - centro[1])**2
                candidatos.append((distancia, punto))
        return min(candidatos)[1] if candidatos else None

    def describir_celda(self, punto, celda):
        """Describe si una posición queda bloqueada por mapa o por margen."""
        posicion = f'({punto[0]:.2f}, {punto[1]:.2f}) m'
        if celda is None:
            return f'{posicion}: fuera del mapa'
        valor = self.datos_mapa[celda]
        distancia = self.distancia_a_pared[celda]
        minimo = self.radio_robot + self.margen_seguridad
        if valor != 0:
            motivo = f'celda ocupada o desconocida (valor={valor})'
        elif not self.libre_fino[celda]:
            motivo = f'bloqueada por margen: distancia={distancia:.2f} m < {minimo:.2f} m'
        else:
            motivo = f'transitable, distancia a obstáculos/desconocido={distancia:.2f} m'
        return f'{posicion}, celda={celda}: {motivo}'

    def punto_para_observar(self, zona, x, y):
        """Busca un destino alcanzable con vista a una celda libre de la zona."""
        self.busqueda_pendiente = False
        comienzo = time.monotonic()
        inicio = self.celda_fina_segura(x, y)
        if inicio is None or not self.libre_fino[inicio]:
            return None
        etiquetas = self.componentes_libres
        alcanzable = etiquetas == etiquetas[inicio]
        if zona in self.puntos_observacion:
            punto = self.puntos_observacion[zona]
            celda = self.celda_fina_segura(*punto)
            if celda is not None and alcanzable[celda]:
                return punto
        alcance = self.ultimo_scan.range_max if self.ultimo_scan is not None else 0.0
        clave = (zona, int(etiquetas[inicio]), alcance)
        if clave not in self.cache_busquedas:
            centro = self.celda_a_mundo(*zona, self.info_grilla)
            f0, c0 = self.celda_de(*centro, self.info_mapa)
            radio = math.ceil(self.info_grilla.resolution / self.info_mapa.resolution) + 1
            alto, ancho = self.libre_fino.shape
            blancos = []
            for f in range(max(0, f0 - radio), min(alto, f0 + radio + 1)):
                for c in range(max(0, c0 - radio), min(ancho, c0 + radio + 1)):
                    if self.datos_mapa[f, c] != 0:
                        continue
                    punto = self.celda_a_mundo(f, c, self.info_mapa)
                    if self.celda_de(*punto, self.info_grilla) == zona:
                        blancos.append((f, c))
            if not blancos:
                return None
            # Se usa el alcance medido del láser, sin suponer visibilidad a través de paredes.
            if not math.isfinite(alcance) or alcance <= 0:
                return None
            candidatos = np.argwhere(alcanzable)
            distancias = (candidatos[:, 0] - f0)**2 + (candidatos[:, 1] - c0)**2
            # Fuera del alcance de toda la zona no puede haber una vista válida.
            cerca = distancias <= (alcance / self.info_mapa.resolution + radio)**2
            candidatos = candidatos[cerca]
            distancias = distancias[cerca]
            orden = np.argsort(distancias)
            if len(self.cache_busquedas) >= 8:
                self.cache_busquedas.pop(next(iter(self.cache_busquedas)))
            self.cache_busquedas[clave] = (blancos, candidatos, orden)
        blancos, candidatos, orden = self.cache_busquedas[clave]
        fallidos = self.observaciones_fallidas.get(zona, [])
        comienzo = time.monotonic()
        desde = self.cursores_observacion.get(clave, 0)
        for posicion in range(desde, len(orden)):
            if posicion > desde and time.monotonic() - comienzo >= 0.025:
                self.cursores_observacion[clave] = posicion
                self.busqueda_pendiente = True
                return None
            if posicion - desde >= 128:
                self.cursores_observacion[clave] = posicion
                self.busqueda_pendiente = True
                return None
            indice = orden[posicion]
            f, c = map(int, candidatos[indice])
            punto = self.celda_a_mundo(f, c, self.info_mapa)
            if math.hypot(punto[0] - x, punto[1] - y) < 0.25:
                continue
            if any(math.hypot(punto[0] - px, punto[1] - py) < 0.25
                   for px, py in fallidos):
                continue
            for blanco in blancos:
                if math.hypot(f - blanco[0], c - blanco[1]) * self.info_mapa.resolution > alcance:
                    continue
                rayo = trazar_rayo(f, c, *blanco)
                if not all(self.datos_mapa[celda] == 0 for celda in rayo):
                    continue
                # Tampoco permitir visibilidad diagonal entre dos paredes.
                if any(a[0] != b[0] and a[1] != b[1] and (
                        self.datos_mapa[a[0], b[1]] != 0
                        or self.datos_mapa[b[0], a[1]] != 0)
                       for a, b in zip(rayo, rayo[1:])):
                    continue
                self.puntos_observacion[zona] = punto
                self.cursores_observacion.pop(clave, None)
                return punto
        self.cursores_observacion.pop(clave, None)
        return None

    def ceder_turno_busqueda(self, zona):
        """Da oportunidad a otras zonas conservando el cursor de búsqueda."""
        turnos = self.turnos_busqueda.get(zona, 0) + 1
        self.turnos_busqueda[zona] = turnos
        if turnos < 10:
            return False
        self.objetivos_pendientes[zona] = (
            self.get_clock().now().nanoseconds / 1e9 + self.reintento_objetivo_s)
        self.turnos_busqueda[zona] = 0
        self.rondas_observacion[zona] = self.rondas_observacion.get(zona, 0) + 1
        return True

    def replanificar_si_hace_falta(self, objetivo_grueso, objetivo_mundo, x_r, y_r):
        """Planifica con A* sobre la grilla fina del /map -- objetivo_grueso
        es la celda de la grilla de cobertura, solo se usa para decidir
        *cuándo* replanificar (cambió la zona elegida); el camino en sí se
        calcula entre las celdas finas del robot y del objetivo."""
        cambio_de_objetivo = objetivo_grueso != self.objetivo_celda
        if cambio_de_objetivo:
            self.get_logger().info(
                f'Nuevo objetivo de cobertura: {objetivo_grueso}; '
                f'destino ({objetivo_mundo[0]:.2f}, {objetivo_mundo[1]:.2f}) m.')
        self.ticks_desde_replan += 1
        toca_por_tiempo = self.ticks_desde_replan >= REPLANIFICAR_CADA_N_TICKS
        sin_camino = not self.camino

        if not (cambio_de_objetivo or toca_por_tiempo or sin_camino):
            return

        self.ticks_desde_replan = 0
        self.objetivo_celda = objetivo_grueso
        celda_robot = self.celda_fina_segura(x_r, y_r)
        celda_objetivo = self.celda_fina_segura(*objetivo_mundo)
        if celda_robot is None or not self.libre_fino[celda_robot]:
            self.get_logger().warn(
                'Planificación detenida: posición estimada del robot '
                + self.describir_celda((x_r, y_r), celda_robot),
                throttle_duration_sec=10.0)
            self.camino = None
            self.publicar_camino([])
            return
        camino = None
        if celda_objetivo is not None and self.libre_fino[celda_objetivo]:
            camino = a_estrella(
                self.libre_fino, celda_robot, celda_objetivo, self.costo_fino)
        if camino is None:
            motivo = self.describir_celda(objetivo_mundo, celda_objetivo)
            if celda_objetivo is not None and self.libre_fino[celda_objetivo]:
                motivo += '; A* no encuentra conexión transitable con el robot'
            self.get_logger().warn(
                f'No hay camino hasta {objetivo_grueso}: objetivo {motivo}. '
                f'Robot en ({x_r:.2f}, {y_r:.2f}) m; se reintentará más adelante.',
                throttle_duration_sec=10.0)
            self.objetivos_pendientes[objetivo_grueso] = (
                self.get_clock().now().nanoseconds / 1e9 + self.reintento_objetivo_s)
            self.camino = None
            self.publicar_camino([])
            return

        self.camino = camino
        self.publicar_camino(camino)

    def siguiente_punto_del_camino(self, x_r, y_r):
        """Descarta del camino los waypoints ya alcanzados y devuelve el
        primero que falta, en coordenadas de `map`. El camino está en
        celdas finas, así que la tolerancia es varias celdas (si fuera una
        sola, el seguimiento sería demasiado fino/lento de recorrer)."""
        if not self.camino:
            return None
        tolerancia = max(self.info_mapa.resolution * 3.0, 0.1)
        celda_robot = self.celda_fina_segura(x_r, y_r)
        while len(self.camino) > 1:
            wx, wy = self.celda_a_mundo(*self.camino[0], self.info_mapa)
            if (math.hypot(wx - x_r, wy - y_r) < tolerancia
                    and segmento_transitable(self.libre_fino, celda_robot, self.camino[1])):
                self.camino.pop(0)
            else:
                break
        return self.celda_a_mundo(*self.camino[0], self.info_mapa)

    # ---------- visualización ----------

    def publicar_objetivo(self, objetivo_mundo):
        marcador = Marker()
        marcador.header.frame_id = self.map_frame
        marcador.header.stamp = self.get_clock().now().to_msg()
        marcador.ns = 'objetivo_actual'
        marcador.id = 0
        marcador.type = Marker.SPHERE
        marcador.action = Marker.ADD
        marcador.pose.position.x = objetivo_mundo[0]
        marcador.pose.position.y = objetivo_mundo[1]
        marcador.pose.orientation.w = 1.0
        marcador.scale.x = marcador.scale.y = marcador.scale.z = 0.25
        marcador.color.r = 1.0
        marcador.color.g = 0.6
        marcador.color.a = 1.0
        self.pub_objetivo.publish(marcador)

    def publicar_camino(self, camino):
        if not camino:
            marcador = Marker()
            marcador.header.frame_id = self.map_frame
            marcador.ns = 'objetivo_actual'
            marcador.id = 0
            marcador.action = Marker.DELETE
            self.pub_objetivo.publish(marcador)
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for fila, col in camino:
            x, y = self.celda_a_mundo(fila, col, self.info_mapa)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub_camino.publish(msg)

    # ---------- máquina de estados ----------

    def transicionar(self, pose):
        """
        TODO: mismo criterio que la máquina de estados de semana 03 --
        primero se decide la transición (esta función), después se actúa
        según el estado ya actualizado (`actuar()`, ya resuelta).

        Completá los 2 `if` de más abajo, uno por estado (el bloque
        `if self.estado != anterior: ...` del final, que loguea cada
        transición, ya está resuelto -- no lo toques).

          - **Si `self.estado == ESTADO_INICIALIZANDO`**: pasar a
            `ESTADO_EXPLORANDO` solo cuando se cumplen TODAS estas
            condiciones a la vez:
              a. `pose is not None` (hay tf).
              b. `self.no_cubiertas is not None` (ya llegó
                 `/grilla_cobertura`).
              c. `self.libre_fino is not None` (ya llegó `/map`).
              d. `self.pose_inicial_confirmada` es `True` (un humano ya
                 clickeó "2D Pose Estimate").
              e. Además, pasó al menos `self.tiempo_asentamiento_s`
                 segundos desde esa confirmación -- calculalo con
                 `(self.get_clock().now() - self.momento_confirmacion).nanoseconds / 1e9`.
            Si se cumplen las 5, cambiá `self.estado` a
            `ESTADO_EXPLORANDO`.
          - **Si `self.estado == ESTADO_EXPLORANDO`**: pasar a
            `ESTADO_TERMINADO` cuando no queda ninguna celda no cubierta
            -- `self.no_cubiertas is not None and not np.any(self.no_cubiertas)`.
        """
        anterior = self.estado
        if self.estado == ESTADO_SIN_DESTINOS:
            if self.no_cubiertas is not None and not np.any(self.no_cubiertas):
                self.estado = ESTADO_TERMINADO
            elif self.no_cubiertas is not None and any(
                    (int(f), int(c)) not in self.zonas_sin_destino
                    for f, c in zip(*np.where(self.no_cubiertas))):
                self.estado = ESTADO_EXPLORANDO
        if self.estado == ESTADO_INICIALIZANDO:

            if (
                pose is not None
                and self.no_cubiertas is not None
                and self.libre_fino is not None
                and self.pose_inicial_confirmada
            ):
                tiempo_transcurrido = (
                    self.get_clock().now() - self.momento_confirmacion
                ).nanoseconds / 1e9

                if tiempo_transcurrido >= self.tiempo_asentamiento_s:
                    self.estado = ESTADO_EXPLORANDO

        elif self.estado == ESTADO_EXPLORANDO:

            if (
                self.no_cubiertas is not None
                and not np.any(self.no_cubiertas)
            ):
                self.estado = ESTADO_TERMINADO
            elif self.no_cubiertas is not None and all(
                    (int(f), int(c)) in self.zonas_sin_destino
                    for f, c in zip(*np.where(self.no_cubiertas))):
                self.estado = ESTADO_SIN_DESTINOS
                self.get_logger().warn(
                    'Quedan zonas sin cubrir, pero no se encontraron más destinos '
                    'ni puntos de observación seguros. No es cobertura completa. '
                    'Se reconsiderarán si cambia el mapa o la grilla.')

        if self.estado != anterior:
            self.get_logger().info(
                f'Transición: {anterior} -> {self.estado}'
            )

    def objetivo_de_la_mision(self, fila_r, col_r, preferidas=None):
        """Celda (gruesa, de la grilla de cobertura) libre no cubierta más
        cercana al robot -- solo se llama en EXPLORANDO, así que siempre
        hay al menos una (si no, transicionar() ya pasó a TERMINADO)."""
        return self.celda_no_cubierta_mas_cercana(fila_r, col_r, preferidas)

    def actuar(self, pose):
        """Movimiento en EXPLORANDO: planifica con A* y sigue el camino
        hasta la zona objetivo."""
        x_r, y_r, yaw_r = pose
        fila_r_gruesa, col_r_gruesa = self.celda_de(x_r, y_r, self.info_grilla)

        celda_robot = self.celda_fina_segura(x_r, y_r)
        componente = self.componentes_libres[celda_robot] if celda_robot is not None else 0
        preferidas = self.zonas_alcanzables(componente)
        objetivo_grueso = self.objetivo_de_la_mision(
            fila_r_gruesa, col_r_gruesa, preferidas)
        if objetivo_grueso is None:
            self.camino = None
            self.seguimiento_avance = None
            self.publicar_camino([])
            self.get_logger().warn(
                'Quedan celdas sin cubrir, pero todos los objetivos están en espera '
                'de reintento por falta de camino o de avance.',
                throttle_duration_sec=10.0)
            self.detener()
            return
        objetivo_mundo = self.punto_seguro_en_zona(objetivo_grueso, componente)
        if objetivo_mundo is not None:
            celda_robot = self.celda_fina_segura(x_r, y_r)
            celda_objetivo = self.celda_fina_segura(*objetivo_mundo)
            if (celda_robot is None or not self.libre_fino[celda_robot]
                    or self.componentes_libres[celda_robot]
                    != self.componentes_libres[celda_objetivo]):
                objetivo_mundo = None
        if objetivo_grueso in self.observaciones_fallidas:
            objetivo_mundo = None
        if objetivo_mundo is None:
            celda_robot = self.celda_fina_segura(x_r, y_r)
            if celda_robot is None or not self.libre_fino[celda_robot]:
                self.get_logger().warn(
                    'No se buscan puntos de observación: posición del robot bloqueada.',
                    throttle_duration_sec=10.0)
                self.camino = None
                self.publicar_camino([])
                self.detener()
                return
            if objetivo_grueso not in self.puntos_observacion:
                self.detener()
            objetivo_mundo = self.punto_para_observar(objetivo_grueso, x_r, y_r)
            if self.busqueda_pendiente:
                if self.ceder_turno_busqueda(objetivo_grueso):
                    self.get_logger().info(
                        f'Búsqueda de {objetivo_grueso} cede el turno a otras zonas; '
                        'se conserva el progreso de búsqueda.')
                self.get_logger().info(
                    f'Buscando punto de observación para {objetivo_grueso} por etapas.',
                    throttle_duration_sec=10.0)
                self.camino = None
                self.publicar_camino([])
                return
        if objetivo_mundo is None:
            self.zonas_sin_destino.add(objetivo_grueso)
            self.get_logger().warn(
                f'Zona {objetivo_grueso} sin punto de observación alcanzable y visible; '
                'se excluye como destino hasta actualizar el mapa. Sigue sin cubrir.')
            self.camino = None
            self.publicar_camino([])
            self.detener()
            return
        self.publicar_objetivo(objetivo_mundo)

        self.replanificar_si_hace_falta(objetivo_grueso, objetivo_mundo, x_r, y_r)

        punto_seguimiento = self.siguiente_punto_del_camino(x_r, y_r)
        if punto_seguimiento is None:
            self.detener()
            return
        if self.objetivo_sin_avance(objetivo_grueso, x_r, y_r):
            self.get_logger().warn(
                f'Objetivo {objetivo_grueso} sin avance durante '
                f'{self.timeout_sin_avance_s:.0f} s; se probará otro objetivo. '
                'La zona sigue pendiente de cobertura.')
            self.objetivos_pendientes[objetivo_grueso] = (
                self.get_clock().now().nanoseconds / 1e9 + self.reintento_objetivo_s)
            self.observaciones_fallidas.setdefault(objetivo_grueso, []).append(objetivo_mundo)
            self.puntos_observacion.pop(objetivo_grueso, None)
            self.seguimiento_avance = None
            self.camino = None
            self.publicar_camino([])
            self.detener()
            return
        distancia_restante = math.hypot(
            punto_seguimiento[0] - x_r, punto_seguimiento[1] - y_r)
        distancia_restante += sum(
            math.hypot(a[0] - b[0], a[1] - b[1]) * self.info_mapa.resolution
            for a, b in zip(self.camino, self.camino[1:]))
        vx_atr, vy_atr, _ = self.vector_hacia(
            x_r, y_r, yaw_r, punto_seguimiento, distancia_restante)
        repulsion = self.vector_repulsivo()
        if repulsion is None:
            self.get_logger().warn(
                'Frenado: scan vencido o transformación del láser no disponible.',
                throttle_duration_sec=10.0)
            self.detener()
            return
        vx_rep, vy_rep = repulsion

        vx, vy = vx_atr + vx_rep, vy_atr + vy_rep
        magnitud = math.hypot(vx, vy)
        if magnitud > self.velocidad_maxima:
            vx *= self.velocidad_maxima / magnitud
            vy *= self.velocidad_maxima / magnitud

        vx = self.suavizado * vx + (1.0 - self.suavizado) * self.vx_anterior
        vy = self.suavizado * vy + (1.0 - self.suavizado) * self.vy_anterior
        self.vx_anterior, self.vy_anterior = vx, vy

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        self.velocidad_ordenada = math.hypot(vx, vy)
        self.pub_cmd_vel.publish(cmd)

    # ---------- control ----------

    def controlar(self):
        pose = self.obtener_pose_robot()

        self.transicionar(pose)
        self.pub_estado.publish(String(data=self.estado))

        if self.estado in (
                ESTADO_INICIALIZANDO, ESTADO_TERMINADO, ESTADO_SIN_DESTINOS,
                ESTADO_DESLIZAMIENTO) or pose is None:
            if pose is None and self.estado == ESTADO_EXPLORANDO:
                self.get_logger().warn(
                    'Frenado: no está disponible la transformación map -> base del robot.',
                    throttle_duration_sec=10.0)
            self.detener()
            return

        if not self.scan_vigente():
            self.get_logger().warn(
                'Frenado: no llega scan o supera timeout_scan_s.',
                throttle_duration_sec=10.0)
            self.detener()
            return
        self.actuar(pose)


def main(args=None):
    rclpy.init(args=args)
    nodo = Explorador()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if nodo.context.ok():
                try:
                    nodo.pub_cmd_vel.publish(Twist())
                except rclpy_implementation.RCLError:
                    # Una señal puede apagar ROS entre la comprobación y el publish.
                    if nodo.context.ok():
                        raise
        finally:
            nodo.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
