"""Visualiza grillas como cubos para evitar el shader de Map de RViz."""

from copy import deepcopy

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def marcador_grilla(msg, cobertura=False):
    """Construye un único marcador con pose y resolución de la grilla."""
    marker = Marker()
    marker.header = msg.header
    marker.ns = 'cobertura' if cobertura else 'mapa'
    marker.id = 0
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.pose = deepcopy(msg.info.origin)
    marker.pose.position.z += 0.012 if cobertura else 0.0
    marker.scale.x = marker.scale.y = msg.info.resolution
    marker.scale.z = 0.01
    marker.color.a = 1.0
    datos = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
    for fila, col in np.argwhere(datos >= 0):
        valor = datos[fila, col]
        marker.points.append(Point(
            x=(float(col) + 0.5) * msg.info.resolution,
            y=(float(fila) + 0.5) * msg.info.resolution, z=0.0))
        if cobertura:
            color = ColorRGBA(r=1.0, g=0.15, b=0.0, a=0.65) if valor else ColorRGBA(
                r=0.0, g=0.8, b=0.2, a=0.65)
        else:
            gris = 1.0 - min(float(valor), 100.0) / 100.0
            color = ColorRGBA(r=gris, g=gris, b=gris, a=1.0)
        marker.colors.append(color)
    return marker


class VisualizadorGrillas(Node):
    """Convierte mapa y cobertura sin cambiar sus tópicos originales."""

    def __init__(self):
        super().__init__('visualizador_grillas')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_mapa = self.create_publisher(Marker, 'mapa_visual', qos)
        self.pub_cobertura = self.create_publisher(Marker, 'cobertura_visual', qos)
        self.create_subscription(OccupancyGrid, 'map', self.recibir_mapa, qos)
        self.create_subscription(OccupancyGrid, 'grilla_cobertura', self.recibir_cobertura, qos)

    def recibir_mapa(self, msg):
        """Publica el mapa con colores de ocupación."""
        self.pub_mapa.publish(marcador_grilla(msg))

    def recibir_cobertura(self, msg):
        """Publica pendientes en rojo y zonas vistas en verde."""
        self.pub_cobertura.publish(marcador_grilla(msg, cobertura=True))


def main(args=None):
    """Ejecuta el visualizador hasta recibir el cierre de ROS."""
    rclpy.init(args=args)
    nodo = VisualizadorGrillas()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()
