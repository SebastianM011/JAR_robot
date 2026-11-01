"""Evasión reactiva de obstáculos con una máquina de estados avanzar/girar."""

import math

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener

from jar_navegacion.corredor import mascara_corredor

FRECUENCIA_HZ = 10.0

ESTADO_AVANZAR = 'avanzar'
ESTADO_GIRAR = 'girar'


def yaw_de_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalizar_angulo(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


class Evasor(Node):
    """
    Máquina de estados avanzar/girar sobre la huella del robot.

    Avanza mientras la huella del robot tenga `distancia_choque_m` libres
    adelante y, si no, gira en el lugar un ángulo fijo medido con la
    odometría. Sin scan, sin TF o con un scan sin datos, frena. Publica en
    cmd_vel: no correrlo junto al explorador ni a Nav2.
    """

    def __init__(self):
        super().__init__('evasor')
        self.declare_parameter('base_frame', 'base_footprint')
        # Misma huella que el explorador: envuelve chasis y ruedas del X3.
        self.declare_parameter('largo_robot', 0.37)
        self.declare_parameter('ancho_robot', 0.24)
        self.declare_parameter('margen_seguridad', 0.05)
        # Recorrido libre que se exige adelante de la huella.
        self.declare_parameter('distancia_choque_m', 0.4)
        self.declare_parameter('velocidad_adelante', 0.3)
        # Positivo = antihorario; siempre se gira hacia el mismo lado.
        self.declare_parameter('velocidad_angular', 1.0)
        self.declare_parameter('angulo_giro_deg', 110.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.largo = self.get_parameter('largo_robot').value
        self.ancho = self.get_parameter('ancho_robot').value
        self.margen = self.get_parameter('margen_seguridad').value
        self.distancia_choque = self.get_parameter('distancia_choque_m').value
        self.velocidad_adelante = self.get_parameter('velocidad_adelante').value
        self.velocidad_angular = self.get_parameter('velocidad_angular').value
        self.angulo_giro = math.radians(self.get_parameter('angulo_giro_deg').value)

        self.ultimo_scan = None
        self.yaw_actual = None
        self.yaw_inicial_giro = 0.0
        self.estado = ESTADO_AVANZAR

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Solo para ver en RViz qué rayos disparan el giro.
        self.publisher_scan_corredor = self.create_publisher(LaserScan, 'scan_corredor', 10)
        self.create_subscription(LaserScan, 'scan', self.recibir_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.recibir_odom, 10)
        self.create_timer(1.0 / FRECUENCIA_HZ, self.maquina_de_estados)

    def recibir_scan(self, msg: LaserScan):
        self.ultimo_scan = msg

    def recibir_odom(self, msg: Odometry):
        self.yaw_actual = yaw_de_quaternion(msg.pose.pose.orientation)

    def corredor_bloqueado(self):
        """Indica si hay obstáculo adelante; None si no se puede decidir."""
        scan = self.ultimo_scan
        if scan is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, scan.header.frame_id, Time())
        except Exception:
            return None
        rangos = np.array(scan.ranges)
        angulos = scan.angle_min + np.arange(len(rangos)) * scan.angle_increment
        t = tf.transform
        mascara = mascara_corredor(rangos, angulos, scan.range_min, scan.range_max,
                                   t.translation.x, t.translation.y,
                                   yaw_de_quaternion(t.rotation), self.largo, self.ancho,
                                   self.margen, self.distancia_choque)
        if mascara is None:
            return None
        filtrado = LaserScan()
        filtrado.header = scan.header
        filtrado.angle_min = scan.angle_min
        filtrado.angle_max = scan.angle_max
        filtrado.angle_increment = scan.angle_increment
        filtrado.time_increment = scan.time_increment
        filtrado.scan_time = scan.scan_time
        filtrado.range_min = scan.range_min
        filtrado.range_max = scan.range_max
        filtrado.ranges = np.where(mascara, rangos, math.inf).tolist()
        self.publisher_scan_corredor.publish(filtrado)
        return bool(mascara.any())

    def maquina_de_estados(self):
        msg = Twist()
        if self.yaw_actual is None:
            self.publisher_.publish(msg)
            return
        if self.estado == ESTADO_AVANZAR:
            bloqueado = self.corredor_bloqueado()
            if bloqueado is None:
                # Sin datos confiables no se avanza ni se gira.
                self.publisher_.publish(msg)
                return
            if bloqueado:
                self.yaw_inicial_giro = self.yaw_actual
                self.estado = ESTADO_GIRAR
                self.get_logger().info('AVANZAR -> GIRAR: obstáculo adelante')
        elif (abs(normalizar_angulo(self.yaw_actual - self.yaw_inicial_giro))
              >= self.angulo_giro):
            self.estado = ESTADO_AVANZAR
            self.get_logger().info('GIRAR -> AVANZAR: giro completo')

        if self.estado == ESTADO_AVANZAR:
            msg.linear.x = self.velocidad_adelante
        else:
            msg.angular.z = self.velocidad_angular
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = Evasor()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
