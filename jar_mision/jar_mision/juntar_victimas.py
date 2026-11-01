"""Reporta en /victimas una posición por cada caja roja detectada."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseArray

from tf2_ros import Buffer, TransformListener

from jar_mision.victimas import AgrupadorVictimas, centro_desde_cara


class JuntarVictimas(Node):
    """
    Escucha /objetos_rojos (detector_obstaculos sobre /scan_rojo): un punto
    en map por cada racha roja, ubicado sobre la cara visible de la caja.
    Lleva cada punto al centro, agrupa por caja y publica la mediana de
    cada grupo una sola vez, con un máximo de `victimas` reportes.
    """

    def __init__(self):
        super().__init__('juntar_victimas')
        self.declare_parameter('victimas', 2)
        self.declare_parameter('detecciones_minimas', 30)
        self.declare_parameter('radio_grupo', 0.30)
        self.declare_parameter('mitad_caja', 0.09)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.agrupador = AgrupadorVictimas(
            self.get_parameter('victimas').value,
            self.get_parameter('detecciones_minimas').value,
            self.get_parameter('radio_grupo').value)
        self.mitad = self.get_parameter('mitad_caja').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseArray, 'objetos_rojos', self.recibir_objetos, 10)
        self.pub = self.create_publisher(PointStamped, 'victimas', 10)

    def posicion_robot(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def recibir_objetos(self, msg: PoseArray):
        robot = self.posicion_robot()
        if robot is None:
            return
        for pose in msg.poses:
            centro = centro_desde_cara(robot, (pose.position.x, pose.position.y), self.mitad)
            if centro is None:
                continue
            reporte = self.agrupador.agregar(*centro)
            if reporte is not None:
                self.publicar(*reporte)

    def publicar(self, x, y):
        punto = PointStamped()
        punto.header.frame_id = self.map_frame
        punto.header.stamp = self.get_clock().now().to_msg()
        punto.point.x = x
        punto.point.y = y
        self.pub.publish(punto)
        self.get_logger().info(
            f'Víctima {self.agrupador.publicados}/{self.agrupador.victimas} '
            f'reportada en ({x:.2f}, {y:.2f}).')


def main(args=None):
    rclpy.init(args=args)
    nodo = JuntarVictimas()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
