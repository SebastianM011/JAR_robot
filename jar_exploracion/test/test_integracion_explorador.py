"""Prueba callbacks y control con mensajes reales y transporte ROS interceptado."""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time
from rclpy.clock import ClockType

from jar_exploracion import explorador
from jar_exploracion.explorador import Explorador


@pytest.fixture
def nodo(monkeypatch):
    """Ejecuta la inicialización real sin DDS, procesos ni movimiento físico."""
    parametros = {}
    monkeypatch.setattr(explorador.Node, '__init__', lambda *args, **kwargs: None)
    monkeypatch.setattr(Explorador, 'declare_parameter',
                        lambda self, name, default: parametros.setdefault(
                            name, SimpleNamespace(value=default)))
    monkeypatch.setattr(Explorador, 'get_parameter', lambda self, name: parametros[name])
    monkeypatch.setattr(Explorador, 'create_publisher', lambda *args, **kwargs: Mock())
    monkeypatch.setattr(Explorador, 'create_subscription', lambda *args, **kwargs: Mock())
    monkeypatch.setattr(Explorador, 'create_timer', lambda *args, **kwargs: Mock())
    reloj = Mock()
    reloj.now.return_value = Time(seconds=100, clock_type=ClockType.ROS_TIME)
    monkeypatch.setattr(Explorador, 'get_clock', lambda self: reloj)
    monkeypatch.setattr(Explorador, 'get_logger', lambda self: Mock())
    buffer = Mock()
    monkeypatch.setattr(explorador, 'Buffer', lambda: buffer)
    monkeypatch.setattr(explorador, 'TransformListener', lambda *args: None)
    n = Explorador()
    yield n, buffer, reloj
    n.ejecutor_plan.shutdown(wait=True, cancel_futures=True)


def mapa():
    """Mapa libre de un metro con marco y metadatos válidos."""
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = msg.info.height = 20
    msg.info.resolution = 0.05
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * 400
    return msg


def test_mapa_repetido_conserva_objetivo_y_mapa_nuevo_frena(nodo):
    """Una retransmisión no interrumpe; un cambio invalida inmediatamente el camino."""
    n, _, _ = nodo
    msg = mapa()
    n.recibir_mapa(msg)
    n.objetivo_mision = (1, 1)
    n.camino = [(10, 10), (10, 11)]
    n.recibir_mapa(msg)
    assert n.objetivo_mision == (1, 1)
    assert n.camino is not None
    msg.data[210] = 100
    n.recibir_mapa(msg)
    assert n.objetivo_mision is None
    assert n.camino is None
    assert not n.libre_fino[n.celda_de(0.525, 0.525, n.info_mapa)]
    assert n.pub_cmd_vel.publish.call_args.args[0].linear.x == 0.0


@pytest.mark.parametrize('defecto', ['frame', 'resolucion', 'datos'])
def test_mapa_invalido_no_sustituye_mapa_bueno(nodo, defecto):
    """Mensajes malformados se rechazan sin destruir el estado existente."""
    n, _, _ = nodo
    n.recibir_mapa(mapa())
    anterior = n.libre_fino.copy()
    msg = mapa()
    if defecto == 'frame':
        msg.header.frame_id = 'otro_robot'
    elif defecto == 'resolucion':
        msg.info.resolution = 0.0
    else:
        msg.data = [0]
    n.recibir_mapa(msg)
    assert np.array_equal(anterior, n.libre_fino)


def test_tf_vencida_o_nan_no_permite_control(nodo):
    """Una posición vieja o no finita no se usa para ordenar movimiento."""
    n, buffer, _ = nodo
    tf = TransformStamped()
    tf.header.stamp = Time(seconds=99).to_msg()
    tf.transform.rotation.w = 1.0
    buffer.lookup_transform.return_value = tf
    assert n.obtener_pose_robot() is None
    tf.header.stamp = Time(seconds=100).to_msg()
    tf.transform.translation.x = float('nan')
    assert n.obtener_pose_robot() is None
    tf.transform.translation.x = 0.5
    assert n.obtener_pose_robot() == (0.5, 0.0, 0.0)


def test_pose_corregida_frena_y_descarta_plan_anterior(nodo):
    """La nueva confirmación no reutiliza un camino de la localización anterior."""
    n, _, _ = nodo
    n.estado = explorador.ESTADO_EXPLORANDO
    n.camino = [(1, 1)]
    n.objetivo_mision = (1, 1)
    n.velocidad_ordenada = 0.2
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    n.recibir_pose_inicial(msg)
    assert n.estado == explorador.ESTADO_INICIALIZANDO
    assert n.camino is None
    assert n.objetivo_mision is None
    assert n.velocidad_ordenada == 0.0


def test_oscilar_no_reinicia_timeout(nodo):
    """Moverse de lado sin reducir el camino no evita el detector de atasco."""
    n, _, reloj = nodo
    assert not n.objetivo_sin_avance((1, 1), 0., 0., 2.0)
    for segundo in range(1, 15):
        reloj.now.return_value = Time(seconds=100 + segundo)
        assert not n.objetivo_sin_avance((1, 1), (-1.)**segundo * 0.1, 0., 2.0)
    reloj.now.return_value = Time(seconds=115)
    assert n.objetivo_sin_avance((1, 1), 0., 0., 2.0)
    assert not n.objetivo_sin_avance((1, 1), 0., 0., 1.9)


def test_control_completo_avanza_y_frena_si_scan_invalido(nodo):
    """El comando publicado pasa por el control final y no sobrevive a perder el láser."""
    from sensor_msgs.msg import LaserScan

    n, buffer, _ = nodo
    msg = mapa()
    n.recibir_mapa(msg)
    cobertura = mapa()
    cobertura.data[10 * 20 + 14] = 100
    n.recibir_grilla(cobertura)
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.header.stamp = Time(seconds=100).to_msg()
    scan.range_min = 0.05
    scan.range_max = 10.0
    scan.angle_increment = 0.1
    scan.ranges = [float('inf')] * 20
    n.recibir_scan(scan)
    tf = TransformStamped()
    tf.transform.rotation.w = 1.0
    buffer.lookup_transform_full.return_value = tf
    n.actuar((0.5, 0.5, 0.0))
    if n.futuro_plan is not None:
        n.futuro_plan.result(timeout=5)
        n.actuar((0.5, 0.5, 0.0))
    comando = n.pub_cmd_vel.publish.call_args.args[0]
    assert 0.0 < comando.linear.x <= 0.3
    assert np.isfinite(comando.linear.y)
    scan.ranges = [float('nan')] * 20
    n.recibir_scan(scan)
    n.actuar((0.5, 0.5, 0.0))
    comando = n.pub_cmd_vel.publish.call_args.args[0]
    assert comando.linear.x == comando.linear.y == 0.0


def test_plan_pendiente_no_parpadea_ni_reinicia_calculo(nodo, monkeypatch):
    """Mantiene el destino durante varios ticks y arranca al completar el mismo futuro."""
    from concurrent.futures import Future
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import Marker
    from jar_exploracion.planificador import a_estrella

    n, buffer, _ = nodo
    n.recibir_mapa(mapa())
    cobertura = mapa()
    cobertura.data[10 * 20 + 14] = 100
    n.recibir_grilla(cobertura)
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.header.stamp = Time(seconds=100).to_msg()
    scan.range_min = 0.05
    scan.range_max = 10.0
    scan.angle_increment = 0.1
    scan.ranges = [float('inf')] * 20
    n.recibir_scan(scan)
    tf = TransformStamped()
    tf.transform.rotation.w = 1.0
    buffer.lookup_transform_full.return_value = tf
    pendiente = Future()
    submit = Mock(return_value=pendiente)
    monkeypatch.setattr(n.ejecutor_plan, 'submit', submit)
    n.pub_objetivo.publish.reset_mock()
    for _ in range(10):
        n.actuar((0.5, 0.5, 0.0))
        assert n.pub_cmd_vel.publish.call_args.args[0].linear.x == 0.0
    assert submit.call_count == 1
    assert all(c.args[0].action == Marker.ADD for c in n.pub_objetivo.publish.call_args_list)
    fin = n.clave_plan[1]
    pendiente.set_result(a_estrella(n.libre_fino, n.celda_de(0.5, 0.5, n.info_mapa), fin))
    n.actuar((0.5, 0.5, 0.0))
    assert n.pub_cmd_vel.publish.call_args.args[0].linear.x > 0.0
    assert submit.call_count == 1


@pytest.mark.parametrize('cubiertas, termina', [(97, False), (98, True), (99, True)])
def test_termina_segun_porcentaje_sin_contar_paredes(nodo, cubiertas, termina):
    """El umbral se calcula sobre libres y no exige visitar cada celda pendiente."""
    n, _, _ = nodo
    n.estado = explorador.ESTADO_EXPLORANDO
    cobertura = mapa()
    cobertura.data = [0] * cubiertas + [100] * (100 - cubiertas) + [-1] * 300
    n.recibir_grilla(cobertura)
    assert n.porcentaje_cobertura() == cubiertas
    n.transicionar((0.5, 0.5, 0.0))
    assert (n.estado == explorador.ESTADO_TERMINADO) == termina


def test_100_por_ciento_opcional_y_grilla_sin_libres(nodo):
    """Cero celdas libres no significa misión completa y 100 conserva el criterio estricto."""
    n, _, _ = nodo
    assert n.porcentaje_cobertura() is None
    n.porcentaje_objetivo = 100.0
    n.estado = explorador.ESTADO_EXPLORANDO
    cobertura = mapa()
    cobertura.data = [0] * 99 + [100] + [-1] * 300
    n.recibir_grilla(cobertura)
    n.transicionar((0.5, 0.5, 0.0))
    assert n.estado == explorador.ESTADO_EXPLORANDO


def test_umbral_cancela_camino_y_frena(nodo, monkeypatch):
    """El siguiente ciclo publica terminado, elimina el destino y deja velocidad cero."""
    from concurrent.futures import Future

    n, _, _ = nodo
    n.estado = explorador.ESTADO_EXPLORANDO
    n.recibir_mapa(mapa())
    cobertura = mapa()
    cobertura.data = [0] * 98 + [100] * 2 + [-1] * 300
    n.recibir_grilla(cobertura)
    n.camino = [(10, 10), (10, 11)]
    n.objetivo_mision = (1, 1)
    n.futuro_plan = Future()
    pendiente = n.futuro_plan
    monkeypatch.setattr(n, 'obtener_pose_robot', lambda: (0.5, 0.5, 0.0))
    n.controlar()
    assert n.estado == explorador.ESTADO_TERMINADO
    assert pendiente.cancelled()
    assert n.camino is None and n.objetivo_mision is None
    assert n.pub_estado.publish.call_args.args[0].data == 'terminado'
    assert n.pub_cmd_vel.publish.call_args.args[0].linear.x == 0.0
