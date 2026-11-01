"""Comprueba geometría y colores sin iniciar el renderizador."""

from nav_msgs.msg import OccupancyGrid

from jar_bringup.visualizador_grillas import marcador_grilla


def test_marcadores_grilla():
    """Respeta resolución, origen, ocupación y celdas desconocidas."""
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = 3
    msg.info.height = 1
    msg.info.resolution = 0.2
    msg.info.origin.position.x = -2.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [0, 100, -1]
    marker = marcador_grilla(msg)
    assert marker.header.frame_id == 'map'
    assert marker.pose.position.x == -2.0
    assert len(marker.points) == len(marker.colors) == 2
    assert abs(marker.points[0].x - 0.1) < 1e-6
    assert marker.colors[0].r == 1.0
    assert marker.colors[1].r == 0.0
    coverage = marcador_grilla(msg, cobertura=True)
    assert coverage.colors[0].g > coverage.colors[0].r
    assert coverage.colors[1].r > coverage.colors[1].g
    assert coverage.pose.position.z > marker.pose.position.z
