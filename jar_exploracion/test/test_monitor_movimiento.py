"""Detecta patinamiento sin confundir la medición real con localización."""

from jar_exploracion.monitor_movimiento import DetectorDeslizamiento


def test_patina_y_estimacion_avanza():
    """Detecta ruedas avanzando mientras el chasis permanece inmóvil."""
    detector = DetectorDeslizamiento()
    for tick in range(30):
        assert not detector.actualizar(tick / 10, (10., 20.), (tick / 100, 0.), 0.2)
    assert detector.actualizar(3., (10., 20.), (0.3, 0.), 0.2)


def test_avance_real_en_otro_marco_no_dispara():
    """Compara desplazamientos, independientemente de origen y orientación."""
    detector = DetectorDeslizamiento()
    for tick in range(61):
        assert not detector.actualizar(
            tick / 10, (10., 20. + tick / 100), (tick / 100, 0.), 0.2)


def test_pausa_salto_y_reinicio_descartan_ventana():
    """No combina muestras separadas por una pausa o reinicio del reloj."""
    detector = DetectorDeslizamiento()
    assert not detector.actualizar(1., (0., 0.), (0., 0.), 0.2)
    assert not detector.actualizar(5., (0., 0.), (1., 0.), 0.2)
    assert not detector.actualizar(0., (0., 0.), (1., 0.), 0.2)
    assert not detector.actualizar(0.1, (0., 0.), (1., 0.), 0.)
    assert detector.inicio is None


def test_deslizamiento_frena_y_bloquea_explorador():
    """La protección detiene comandos y no publica una pose ficticia."""
    from unittest.mock import Mock
    from nav_msgs.msg import Odometry
    from jar_exploracion.explorador import (
        ESTADO_DESLIZAMIENTO, ESTADO_EXPLORANDO, Explorador)

    nodo = Explorador.__new__(Explorador)
    nodo.estado = ESTADO_EXPLORANDO
    nodo.detector_deslizamiento = Mock()
    nodo.detector_deslizamiento.actualizar.return_value = True
    nodo.obtener_pose_robot = lambda: (1., 2., 0.)
    nodo.velocidad_ordenada = 0.2
    nodo.get_logger = lambda: Mock()
    nodo.publicar_camino = Mock()
    nodo.pub_cmd_vel = Mock()
    nodo.verificar_movimiento(Odometry())
    assert nodo.estado == ESTADO_DESLIZAMIENTO
    assert nodo.velocidad_ordenada == 0.0
    assert nodo.camino is None
    assert nodo.pub_cmd_vel.publish.call_args.args[0].linear.x == 0.0
