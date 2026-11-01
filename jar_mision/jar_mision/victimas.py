"""Agrupamiento de detecciones de cajas rojas, sin dependencias de ROS."""

import math

import numpy as np


def centro_desde_cara(robot, deteccion, mitad_caja):
    """
    Corre la detección de la cara visible media caja hacia adentro.

    Devuelve None si la detección coincide con la posición del robot.
    """
    dx = deteccion[0] - robot[0]
    dy = deteccion[1] - robot[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return None
    return deteccion[0] + mitad_caja * dx / d, deteccion[1] + mitad_caja * dy / d


class AgrupadorVictimas:
    """
    Junta detecciones cercanas y decide cuándo reportar cada grupo.

    La mediana del grupo ignora las pocas detecciones que caen lejos de la
    caja, que con un promedio la desplazarían.
    """

    def __init__(self, victimas=2, detecciones_minimas=30, radio_grupo=0.30):
        self.victimas = victimas
        self.minimas = detecciones_minimas
        self.radio = radio_grupo
        self.grupos = []
        self.publicados = 0

    def agregar(self, x, y):
        """Suma el punto a su grupo; devuelve la posición a reportar o None."""
        for grupo in self.grupos:
            centro = np.median(np.array(grupo['puntos']), axis=0)
            if math.hypot(x - centro[0], y - centro[1]) < self.radio:
                grupo['puntos'].append((x, y))
                return self.listo(grupo)
        grupo = {'puntos': [(x, y)], 'publicado': False}
        self.grupos.append(grupo)
        return self.listo(grupo)

    def listo(self, grupo):
        """Marca el grupo como publicado si alcanzó las detecciones mínimas."""
        if (grupo['publicado'] or self.publicados >= self.victimas
                or len(grupo['puntos']) < self.minimas):
            return None
        grupo['publicado'] = True
        self.publicados += 1
        x, y = np.median(np.array(grupo['puntos']), axis=0)
        return float(x), float(y)
