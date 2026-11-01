"""Obstáculos en el recorrido frontal de la huella del robot, sin ROS."""

import numpy as np

from jar_exploracion.seguridad import distancia_huella


def mascara_corredor(rangos, angulos, range_min, range_max, tx, ty, yaw,
                     largo, ancho, margen, distancia):
    """
    True en los rayos que chocarían si la huella avanza `distancia` en +x.

    El scan se lleva a base_footprint con (tx, ty, yaw). Devuelve None si no
    hay ninguna medición ni rayos sin retorno: eso no demuestra espacio libre.
    """
    rangos = np.asarray(rangos, dtype=float)
    validos = (np.isfinite(rangos) & (rangos > 0)
               & (rangos >= range_min) & (rangos <= range_max))
    if not validos.any() and not np.isposinf(rangos).any():
        return None
    r = np.where(validos, rangos, 0.0)
    a = np.asarray(angulos) + yaw
    puntos = np.column_stack((tx + r * np.cos(a), ty + r * np.sin(a)))
    # La huella barrida es una caja desde la posición actual hasta `distancia`.
    barrida = distancia_huella(puntos - np.array([distancia / 2.0, 0.0]),
                               largo + distancia, ancho)
    return validos & (barrida < margen)
