"""Verifica objetivos seguros sin reducir el margen ni salir de la zona."""

import math

import numpy as np
import pytest

pytest.importorskip('rclpy')
from nav_msgs.msg import MapMetaData

from jar_exploracion.explorador import Explorador


@pytest.mark.parametrize('yaw', [0.0, 0.7])
def test_objetivo_en_zona_con_centro_bloqueado(yaw):
    """Elige una celda segura del bloque incluso con origen rotado."""
    nodo = Explorador.__new__(Explorador)
    nodo.info_mapa = MapMetaData()
    nodo.info_mapa.resolution = 0.05
    nodo.info_mapa.origin.position.x = -0.981
    nodo.info_mapa.origin.position.y = -0.956
    nodo.info_mapa.origin.orientation.z = math.sin(yaw / 2)
    nodo.info_mapa.origin.orientation.w = math.cos(yaw / 2)
    nodo.info_grilla = MapMetaData()
    nodo.info_grilla.resolution = 0.2
    nodo.info_grilla.origin = nodo.info_mapa.origin
    nodo.libre_fino = np.zeros((8, 8), dtype=bool)
    nodo.libre_fino[0, 1] = True
    nodo.libre_fino[4, 4] = True
    punto = nodo.punto_seguro_en_zona((0, 0))
    assert nodo.celda_de(*punto, nodo.info_mapa) == (0, 1)
    assert nodo.celda_de(*punto, nodo.info_grilla) == (0, 0)
    nodo.libre_fino[0, 1] = False
    assert nodo.punto_seguro_en_zona((0, 0)) is None
    assert nodo.punto_seguro_en_zona((10, 10)) is None


def test_timeout_avance():
    """Reintenta un objetivo quieto y renueva el plazo si hay desplazamiento."""
    from unittest.mock import Mock

    nodo = Explorador.__new__(Explorador)
    reloj = Mock()
    nodo.get_clock = lambda: reloj
    nodo.timeout_sin_avance_s = 15.0
    nodo.seguimiento_avance = None

    def evaluar(tiempo, x, objetivo=(1, 1)):
        reloj.now.return_value.nanoseconds = int(tiempo * 1e9)
        return nodo.objetivo_sin_avance(objetivo, x, 0.0)

    assert not evaluar(0, 0.0)
    assert not evaluar(14, 0.01)
    assert evaluar(15, 0.01)
    assert not evaluar(16, 0.1)
    assert not evaluar(30, 0.1)
    assert evaluar(31, 0.1)
    assert not evaluar(32, 0.1, (2, 2))


def test_zonas_sin_destino_no_son_cobertura_completa():
    """Detiene reintentos geométricos sin declarar cubiertas las zonas."""
    from unittest.mock import Mock
    from jar_exploracion.explorador import (
        ESTADO_EXPLORANDO, ESTADO_SIN_DESTINOS, ESTADO_TERMINADO)

    nodo = Explorador.__new__(Explorador)
    nodo.get_logger = lambda: Mock()
    nodo.estado = ESTADO_EXPLORANDO
    nodo.no_cubiertas = np.array([[True, False]])
    nodo.zonas_sin_destino = {(0, 0)}
    nodo.transicionar((0., 0., 0.))
    assert nodo.estado == ESTADO_SIN_DESTINOS
    assert nodo.no_cubiertas[0, 0]
    nodo.zonas_sin_destino.clear()
    nodo.transicionar((0., 0., 0.))
    assert nodo.estado == ESTADO_EXPLORANDO
    nodo.no_cubiertas[:] = False
    nodo.transicionar((0., 0., 0.))
    assert nodo.estado == ESTADO_TERMINADO


def test_observar_zona_sin_entrar_y_sin_atravesar_pared(monkeypatch):
    """El punto pertenece al componente del robot y tiene visión libre."""
    from types import SimpleNamespace

    nodo = Explorador.__new__(Explorador)
    nodo.info_mapa = MapMetaData()
    nodo.info_mapa.resolution = 0.1
    nodo.info_mapa.origin.orientation.w = 1.0
    nodo.info_grilla = MapMetaData()
    nodo.info_grilla.resolution = 0.2
    nodo.info_grilla.origin.orientation.w = 1.0
    nodo.datos_mapa = np.zeros((10, 10), dtype=int)
    nodo.libre_fino = np.zeros((10, 10), dtype=bool)
    nodo.libre_fino[1:9, 1:4] = True
    nodo.libre_fino[5, 7] = True  # Isla segura pero desconectada del robot.
    from scipy import ndimage

    nodo.componentes_libres, _ = ndimage.label(nodo.libre_fino)
    nodo.cursores_observacion = {}
    nodo.cache_busquedas = {}
    nodo.ultimo_scan = SimpleNamespace(range_max=2.0)
    nodo.puntos_observacion = {}
    nodo.observaciones_fallidas = {}
    punto = nodo.punto_para_observar((2, 3), 0.15, 0.15)
    assert punto is not None
    celda = nodo.celda_de(*punto, nodo.info_mapa)
    assert nodo.libre_fino[celda]
    assert celda[1] < 4
    nodo.datos_mapa[:, 5] = 100
    nodo.puntos_observacion.clear()
    assert nodo.punto_para_observar((2, 3), 0.15, 0.15) is None
    # Forzar el presupuesto temporal: debe continuar sin volver al inicio.
    import itertools
    from jar_exploracion import explorador

    tiempos = itertools.count(step=0.03)
    monkeypatch.setattr(explorador.time, 'monotonic', lambda: next(tiempos))
    intentos = 0
    while True:
        assert nodo.punto_para_observar((2, 3), 0.15, 0.15) is None
        intentos += 1
        if not nodo.busqueda_pendiente:
            break
        assert intentos < 100
    assert intentos > 1
    assert nodo.cursores_observacion == {}


def test_velocidad_segun_recorrido_restante():
    """No frena en cada waypoint cercano, pero sí al terminar el recorrido."""
    nodo = Explorador.__new__(Explorador)
    nodo.ganancia_atractiva = 0.6
    vx, vy, distancia = nodo.vector_hacia(0., 0., 0., (0.1, 0.), 1.0)
    assert vx == pytest.approx(0.6)
    assert vy == 0.0
    assert distancia == pytest.approx(0.1)
    vx_final, _, _ = nodo.vector_hacia(0., 0., 0., (0.1, 0.), 0.1)
    assert vx_final == pytest.approx(0.06)
    assert nodo.vector_hacia(0., 0., 0., (0., 0.), 1.0) == (0., 0., 0.)


def test_busqueda_costosa_cede_turno_sin_perder_cursor():
    """Una búsqueda pendiente no monopoliza la elección de objetivos."""
    from unittest.mock import Mock

    nodo = Explorador.__new__(Explorador)
    reloj = Mock()
    reloj.now.return_value.nanoseconds = int(20e9)
    nodo.get_clock = lambda: reloj
    nodo.turnos_busqueda = {}
    nodo.rondas_observacion = {}
    nodo.objetivos_pendientes = {}
    nodo.reintento_objetivo_s = 10.0
    clave = ((2, 3), 1, 5.0)
    nodo.cursores_observacion = {clave: 128}
    for _ in range(9):
        assert not nodo.ceder_turno_busqueda((2, 3))
    assert nodo.ceder_turno_busqueda((2, 3))
    assert nodo.objetivos_pendientes[(2, 3)] == 30.0
    assert nodo.cursores_observacion[clave] == 128


def test_priorizar_destinos_directos_y_rotar_busquedas():
    """Una zona cercana difícil no bloquea zonas navegables ni zonas aún no probadas."""
    from unittest.mock import Mock

    nodo = Explorador.__new__(Explorador)
    reloj = Mock()
    reloj.now.return_value.nanoseconds = int(100e9)
    nodo.get_clock = lambda: reloj
    nodo.no_cubiertas = np.array([[True, False, True, True]])
    nodo.zonas_sin_destino = set()
    nodo.objetivos_pendientes = {}
    nodo.objetivo_mision = None
    nodo.rondas_observacion = {(0, 0): 1}
    nodo.observaciones_fallidas = {}
    preferidas = np.array([[False, False, False, True]])
    assert nodo.celda_no_cubierta_mas_cercana(0, 0, preferidas) == (0, 3)
    preferidas[:] = False
    nodo.objetivo_mision = None  # Simular fin de la etapa de navegación directa.
    assert nodo.celda_no_cubierta_mas_cercana(0, 0, preferidas) == (0, 2)
    nodo.no_cubiertas[0, 2] = False
    assert nodo.celda_no_cubierta_mas_cercana(0, 0, preferidas) == (0, 3)


def test_camino_descartado_borra_marcador_objetivo():
    """El objetivo anterior no permanece visible durante la búsqueda."""
    from unittest.mock import Mock
    from visualization_msgs.msg import Marker

    nodo = Explorador.__new__(Explorador)
    nodo.map_frame = 'map'
    nodo.get_clock = lambda: Mock()
    nodo.pub_objetivo = Mock()
    nodo.pub_camino = Mock()
    # El timestamp necesita un mensaje real.
    from rclpy.clock import Clock
    nodo.get_clock = lambda: Clock()
    nodo.publicar_camino([])
    marcador = nodo.pub_objetivo.publish.call_args.args[0]
    assert marcador.action == Marker.DELETE
    assert marcador.ns == 'objetivo_actual'
    assert nodo.pub_camino.publish.call_args.args[0].poses == []


def test_proyectar_zonas_alcanzables_rotadas():
    """Solo prioriza zonas con celdas finas conectadas al robot."""
    nodo = Explorador.__new__(Explorador)
    nodo.info_mapa = MapMetaData()
    nodo.info_mapa.resolution = 0.05
    nodo.info_mapa.origin.position.x = -1.0
    nodo.info_mapa.origin.orientation.z = math.sin(0.7 / 2)
    nodo.info_mapa.origin.orientation.w = math.cos(0.7 / 2)
    nodo.info_grilla = MapMetaData()
    nodo.info_grilla.resolution = 0.2
    nodo.info_grilla.origin = nodo.info_mapa.origin
    nodo.no_cubiertas = np.ones((2, 2), dtype=bool)
    nodo.componentes_libres = np.zeros((8, 8), dtype=int)
    nodo.componentes_libres[0, 1] = 1
    nodo.componentes_libres[6, 6] = 2
    nodo.cache_zonas_alcanzables = {}
    assert np.array_equal(nodo.zonas_alcanzables(1), [[True, False], [False, False]])
    assert not nodo.zonas_alcanzables(0).any()


def test_seguimiento_no_corta_esquina_por_tolerancia():
    """Conserva el waypoint que evita rozar una esquina inflada."""
    nodo = Explorador.__new__(Explorador)
    nodo.info_mapa = MapMetaData()
    nodo.info_mapa.resolution = 0.1
    nodo.info_mapa.origin.orientation.w = 1.0
    nodo.libre_fino = np.array([[True, False], [True, True]])
    nodo.camino = [(1, 0), (1, 1)]
    punto = nodo.siguiente_punto_del_camino(0.05, 0.05)
    assert nodo.camino == [(1, 0), (1, 1)]
    assert punto == pytest.approx((0.05, 0.15))
    nodo.libre_fino[0, 1] = True
    nodo.siguiente_punto_del_camino(0.05, 0.05)
    assert nodo.camino == [(1, 1)]


def test_objetivo_persistente_hasta_cubrir_o_rechazar():
    """Cruzar la mitad entre dos zonas no debe invertir el objetivo cada tick."""
    from unittest.mock import Mock

    nodo = Explorador.__new__(Explorador)
    reloj = Mock()
    reloj.now.return_value.nanoseconds = int(100e9)
    nodo.get_clock = lambda: reloj
    nodo.no_cubiertas = np.array([[True, False, False, False, True]])
    nodo.zonas_sin_destino = set()
    nodo.objetivos_pendientes = {}
    nodo.rondas_observacion = {}
    nodo.observaciones_fallidas = {}
    nodo.objetivo_mision = None
    preferidas = nodo.no_cubiertas.copy()
    assert nodo.celda_no_cubierta_mas_cercana(0, 0, preferidas) == (0, 0)
    for columna in (4, 0, 3, 1, 4):
        assert nodo.celda_no_cubierta_mas_cercana(0, columna, preferidas) == (0, 0)
    nodo.no_cubiertas[0, 0] = False
    assert nodo.celda_no_cubierta_mas_cercana(0, 0, preferidas) == (0, 4)
    nodo.no_cubiertas[0, 0] = True
    nodo.objetivos_pendientes[(0, 4)] = 110.0
    assert nodo.celda_no_cubierta_mas_cercana(0, 4, preferidas) == (0, 0)
    # Un objetivo geométricamente descartado también se libera.
    nodo.zonas_sin_destino.add((0, 0))
    assert nodo.celda_no_cubierta_mas_cercana(0, 0, preferidas) is None
    assert nodo.objetivo_mision is None
