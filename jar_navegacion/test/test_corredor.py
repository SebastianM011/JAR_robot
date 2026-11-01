"""Pruebas del corredor del evasor sin ejecutar ROS."""

import math

import numpy as np

from jar_navegacion.corredor import mascara_corredor

LARGO, ANCHO, MARGEN = 0.37, 0.24, 0.05


def corredor(rangos, angulos, yaw=0.0, distancia=0.4):
    return mascara_corredor(np.array(rangos), np.array(angulos), 0.05, 12.0,
                            0.0, 0.0, yaw, LARGO, ANCHO, MARGEN, distancia)


def test_obstaculo_adelante_y_no_atras():
    m = corredor([0.5, 0.5], [0.0, math.pi])
    assert m.tolist() == [True, False]


def test_respeta_el_ancho_de_la_huella():
    """Un punto a 0.16 m del eje roza la huella con margen; a 0.30 m pasa."""
    for lateral, choca in [(0.16, True), (0.30, False)]:
        x, y = 0.4, lateral
        m = corredor([math.hypot(x, y)], [math.atan2(y, x)])
        assert m.tolist() == [choca]


def test_mas_alla_de_la_distancia_no_cuenta():
    assert not corredor([1.0], [0.0], distancia=0.4).any()


def test_lidar_girado_180():
    """Con el lidar girado, el 0 del scan apunta hacia atrás del robot."""
    m = corredor([0.4, 0.4], [0.0, math.pi], yaw=math.pi)
    assert m.tolist() == [False, True]


def test_scan_sin_datos_no_es_espacio_libre():
    assert corredor([float('nan')] * 3, [0.0, 0.1, 0.2]) is None


def test_scan_sin_retorno_es_libre():
    assert not corredor([float('inf')] * 2, [0.0, 0.1]).any()
