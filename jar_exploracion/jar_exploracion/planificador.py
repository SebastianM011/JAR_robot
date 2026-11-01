"""Planificación A* sobre una grilla de celdas transitables."""

import heapq
import math

import numpy as np

from jar_exploracion.trazado import trazar_rayo


def segmento_transitable(libres, inicio, fin):
    """Verifica un segmento sin salir del mapa ni cortar esquinas bloqueadas."""
    if inicio is None or fin is None:
        return False
    alto, ancho = libres.shape
    celdas = trazar_rayo(*inicio, *fin)
    for f, c in celdas:
        if not (0 <= f < alto and 0 <= c < ancho) or not libres[f, c]:
            return False
    for a, b in zip(celdas, celdas[1:]):
        if a[0] != b[0] and a[1] != b[1]:
            if not libres[a[0], b[1]] or not libres[b[0], a[1]]:
                return False
    return True


def a_estrella(libres, inicio, objetivo, costos=None):
    """Devuelve el camino incluyendo sus extremos, o None si no existe."""
    libres = np.asarray(libres, dtype=bool)
    if libres.ndim != 2:
        raise ValueError('La grilla debe tener dos dimensiones')
    if costos is None:
        costos = np.zeros(libres.shape, dtype=float)
    else:
        costos = np.asarray(costos, dtype=float)
    if costos.shape != libres.shape or not np.all(np.isfinite(costos)):
        raise ValueError('Los costos deben ser finitos y tener la forma de la grilla')
    if np.any(costos < 0):
        raise ValueError('Los costos no pueden ser negativos')

    alto, ancho = libres.shape

    def valida(celda):
        return (0 <= celda[0] < alto and 0 <= celda[1] < ancho
                and libres[celda])

    if inicio is None or objetivo is None:
        return None
    inicio, objetivo = tuple(inicio), tuple(objetivo)
    if not valida(inicio) or not valida(objetivo):
        return None

    def heuristica(celda):
        return math.hypot(celda[0] - objetivo[0], celda[1] - objetivo[1])

    pendientes = [(heuristica(inicio), 0.0, inicio)]
    distancias = {inicio: 0.0}
    anteriores = {}
    vecinos = [(df, dc) for df in (-1, 0, 1) for dc in (-1, 0, 1)
               if df != 0 or dc != 0]
    while pendientes:
        _, distancia, actual = heapq.heappop(pendientes)
        if distancia > distancias[actual]:
            continue
        if actual == objetivo:
            camino = [actual]
            while actual in anteriores:
                actual = anteriores[actual]
                camino.append(actual)
            return camino[::-1]
        fila, col = actual
        for df, dc in vecinos:
            vecino = (fila + df, col + dc)
            if not valida(vecino):
                continue
            # Las diagonales no pueden atravesar las esquinas de obstáculos.
            if df and dc and (not libres[fila + df, col] or not libres[fila, col + dc]):
                continue
            nueva = distancia + math.hypot(df, dc) * (1.0 + costos[vecino])
            if nueva < distancias.get(vecino, math.inf):
                distancias[vecino] = nueva
                anteriores[vecino] = actual
                heapq.heappush(pendientes, (nueva + heuristica(vecino), nueva, vecino))
    return None
