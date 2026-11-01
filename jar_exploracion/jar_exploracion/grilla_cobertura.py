import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

from tf2_ros import Buffer, TransformListener

from jar_exploracion.trazado import trazar_rayo

FRECUENCIA_HZ = 10.0


def yaw_de_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GrillaCobertura(Node):
    """
    Construye, a partir del mapa estático de map_server, una segunda
    grilla propia de "cubierto / no cubierto", a una resolución más gruesa
    (factor_escala celdas del mapa original por lado, por cada celda de
    cobertura). Por cada rayo del /scan, traza la línea (Bresenham) entre
    la celda del robot y la celda donde terminó el rayo, y marca como
    cubiertas todas las celdas libres que esa línea atraviesa — no un
    radio fijo alrededor del robot, que ignoraría las paredes y marcaría
    como "visto" lo que en realidad está oculto detrás de un obstáculo.

    Publica la grilla resultante como su propio OccupancyGrid
    (`grilla_cobertura`): 0 = celda libre ya cubierta, 100 = celda libre
    todavía sin cubrir, -1 = celda ocupada o fuera del mapa (no se
    dibuja). Con "Color Scheme: costmap" en el display de Map de RViz,
    eso se ve como una zona que pasa de rojo a verde a medida que el
    robot la recorre.

    No marca nada hasta que un humano confirma la pose inicial con "2D
    Pose Estimate" en RViz (mismo tópico `initialpose` que usa
    `localizador` para re-sembrar el filtro de partículas) -- si se
    empieza a marcar cobertura con una pose todavía sin converger, esas
    marcas quedan mal para siempre: a diferencia de la pose del robot, la
    cobertura no se corrige sola después.
    """

    def __init__(self):
        super().__init__('grilla_cobertura')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('factor_escala', 4)
        self.declare_parameter('distancia_minima_valida', 0.0)
        self.declare_parameter('timeout_scan_s', 0.5)
        self.declare_parameter('tiempo_asentamiento_s', 3.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.factor_escala = self.get_parameter('factor_escala').value
        self.distancia_minima_valida = self.get_parameter('distancia_minima_valida').value

        self.timeout_scan_s = self.get_parameter('timeout_scan_s').value
        self.tiempo_asentamiento_s = self.get_parameter('tiempo_asentamiento_s').value
        if self.factor_escala < 1:
            raise ValueError('factor_escala debe ser positivo')
        if self.timeout_scan_s <= 0 or self.tiempo_asentamiento_s < 0:
            raise ValueError('Los tiempos configurados no son válidos')
        self.momento_confirmacion = None
        self.momento_scan = None
        self.scan_procesado = None
        self.info_mapa = None
        self.libre_fino = None

        # Se completan al recibir /map (recibir_mapa)
        self.libre_c = None       # bool[alto_c, ancho_c]: la celda es transitable
        self.cubierta = None      # bool[alto_c, ancho_c]: la celda ya fue vista
        self.alto_c = self.ancho_c = 0
        self.resolucion_c = 0.0
        self.origen_x = self.origen_y = 0.0
        self.orientacion_origen = Quaternion()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.aviso_tf_mostrado = False
        self.pose_inicial_confirmada = False
        self.ultimo_scan = None

        qos_mapa = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, 'map', self.recibir_mapa, qos_mapa)
        self.create_subscription(LaserScan, 'scan', self.recibir_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.recibir_pose_inicial, 10)
        self.pub_grilla = self.create_publisher(OccupancyGrid, 'grilla_cobertura', qos_mapa)
        self.timer = self.create_timer(1.0 / FRECUENCIA_HZ, self.marcar_cobertura)

    # ---------- construcción de la grilla ----------

    def recibir_mapa(self, msg: OccupancyGrid):
        alto, ancho = msg.info.height, msg.info.width
        datos = np.array(msg.data, dtype=np.int16).reshape((alto, ancho))
        libre_fino = (datos == 0)

        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn('El frame del mapa no coincide con map_frame.')
            return
        if self.info_mapa == msg.info and np.array_equal(self.libre_fino, libre_fino):
            return
        self.info_mapa = msg.info
        self.libre_fino = libre_fino
        self.scan_procesado = None
        factor = self.factor_escala
        self.alto_c = math.ceil(alto / factor)
        self.ancho_c = math.ceil(ancho / factor)
        recorte = np.zeros((self.alto_c * factor, self.ancho_c * factor), dtype=bool)
        recorte[:alto, :ancho] = libre_fino
        bloques = recorte.reshape(self.alto_c, factor, self.ancho_c, factor)
        # Una celda de cobertura es "libre" si la mayoría de las celdas
        # finas que agrupa lo son — evita que una sola celda ocupada en el
        # borde de un pasillo angosto marque todo el bloque como pared.
        self.libre_c = bloques.mean(axis=(1, 3)) > 0.5
        self.cubierta = np.zeros((self.alto_c, self.ancho_c), dtype=bool)

        self.resolucion_c = msg.info.resolution * factor
        self.origen_x = msg.info.origin.position.x
        self.origen_y = msg.info.origin.position.y
        self.orientacion_origen = msg.info.origin.orientation

        self.get_logger().info(
            f'Grilla de cobertura: {self.alto_c}x{self.ancho_c} celdas de '
            f'{self.resolucion_c:.2f} m ({int(np.sum(self.libre_c))} celdas libres).')
        self.publicar_grilla()

    def celda_de(self, x: float, y: float):
        yaw = yaw_de_quaternion(self.orientacion_origen)
        c, sn = math.cos(yaw), math.sin(yaw)
        dx, dy = x - self.origen_x, y - self.origen_y
        return (math.floor((-sn * dx + c * dy) / self.resolucion_c),
                math.floor((c * dx + sn * dy) / self.resolucion_c))

    def dentro_de_grilla(self, fila: int, col: int) -> bool:
        return 0 <= fila < self.alto_c and 0 <= col < self.ancho_c

    # ---------- callbacks ----------

    def recibir_pose_inicial(self, msg):
        # Si ya se había confirmado antes (el humano corrigió la pose a
        # mitad de recorrido), la cobertura marcada hasta ahora quedó
        # calculada con una localización que ya no se considera buena --
        # se vuelve a empezar de cero en vez de arrastrar marcas dudosas.
        if self.cubierta is not None:
            self.cubierta[:] = False
            self.publicar_grilla()
        self.momento_confirmacion = self.get_clock().now()
        self.scan_procesado = self.ultimo_scan
        self.pose_inicial_confirmada = True
        self.get_logger().info('Pose confirmada; esperando el asentamiento.')

    def obtener_pose_robot(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception as e:
            if not self.aviso_tf_mostrado:
                self.get_logger().warn(
                    f'Todavía no hay tf {self.base_frame}->{self.map_frame}: {e}')
                self.aviso_tf_mostrado = True
            return None
        t = tf.transform.translation
        return t.x, t.y

    def recibir_scan(self, msg: LaserScan):
        """Guarda el scan para procesarlo una sola vez desde el timer."""
        self.ultimo_scan = msg
        self.momento_scan = self.get_clock().now()

    def marcar_cobertura(self):
        """Marca visibilidad sobre el mapa fino sin atravesar obstáculos."""
        msg = self.ultimo_scan
        if (not self.pose_inicial_confirmada or self.cubierta is None
                or msg is None or msg is self.scan_procesado):
            return
        ahora = self.get_clock().now()
        asentamiento = (ahora - self.momento_confirmacion).nanoseconds / 1e9
        if asentamiento < self.tiempo_asentamiento_s:
            return
        edad = (ahora - self.momento_scan).nanoseconds / 1e9
        if not 0.0 <= edad <= self.timeout_scan_s:
            return
        instante = Time.from_msg(msg.header.stamp)
        if instante.nanoseconds:
            edad = (ahora - instante).nanoseconds / 1e9
            if not 0.0 <= edad <= self.timeout_scan_s:
                return
        if not math.isfinite(msg.range_max) or msg.range_max <= 0:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, instante)
        except Exception:
            return
        self.scan_procesado = msg
        tf = transform.transform
        tx, ty = tf.translation.x, tf.translation.y
        yaw = yaw_de_quaternion(tf.rotation)
        origen_yaw = yaw_de_quaternion(self.orientacion_origen)
        c, sn = math.cos(origen_yaw), math.sin(origen_yaw)
        resolucion = self.info_mapa.resolution

        def celda_fina(x, y):
            dx, dy = x - self.origen_x, y - self.origen_y
            return (math.floor((-sn * dx + c * dy) / resolucion),
                    math.floor((c * dx + sn * dy) / resolucion))

        inicio = celda_fina(tx, ty)
        alto, ancho = self.libre_fino.shape
        if not (0 <= inicio[0] < alto and 0 <= inicio[1] < ancho):
            return
        hubo_cambios = False
        minimo = max(msg.range_min, self.distancia_minima_valida)
        for i, rango in enumerate(msg.ranges):
            if math.isnan(rango) or rango < minimo or rango <= 0:
                continue
            alcance = min(rango, msg.range_max)
            angulo = yaw + msg.angle_min + i * msg.angle_increment
            fin = celda_fina(tx + alcance * math.cos(angulo),
                             ty + alcance * math.sin(angulo))
            for fila, col in trazar_rayo(*inicio, *fin):
                if not (0 <= fila < alto and 0 <= col < ancho):
                    break
                if not self.libre_fino[fila, col]:
                    break
                fc, cc = fila // self.factor_escala, col // self.factor_escala
                if self.libre_c[fc, cc] and not self.cubierta[fc, cc]:
                    self.cubierta[fc, cc] = True
                    hubo_cambios = True
        if hubo_cambios:
            self.publicar_grilla()

    # ---------- publisher ----------

    def publicar_grilla(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolucion_c
        msg.info.width = self.ancho_c
        msg.info.height = self.alto_c
        msg.info.origin.position.x = self.origen_x
        msg.info.origin.position.y = self.origen_y
        msg.info.origin.orientation = self.orientacion_origen

        datos = np.full((self.alto_c, self.ancho_c), -1, dtype=np.int8)
        datos[self.libre_c & self.cubierta] = 0
        datos[self.libre_c & ~self.cubierta] = 100
        msg.data = datos.flatten().tolist()
        self.pub_grilla.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = GrillaCobertura()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
