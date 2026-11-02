"""Giro de 360° por pasos para que la cámara frontal mire alrededor, sin ROS."""

import math

import numpy as np


def normalizar(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


def radio_de_giro(largo, ancho, margen):
    """Radio del círculo que barre la huella rectangular al girar en el lugar."""
    return math.hypot(largo / 2.0, ancho / 2.0) + margen


def giro_seguro(impactos, distancia_a_pared, radio):
    """
    Indica si el robot puede girar en el lugar.

    `impactos` son puntos del scan en base_footprint (Nx2); `distancia_a_pared`
    es la distancia del centro del robot al obstáculo más cercano del mapa.
    """
    if distancia_a_pared is None or distancia_a_pared < radio:
        return False
    puntos = np.asarray(impactos, dtype=float).reshape(-1, 2)
    return not len(puntos) or float(np.min(np.hypot(puntos[:, 0], puntos[:, 1]))) >= radio


class MiradaPorPasos:
    """
    Gira 360° en `pasos` tramos iguales y se detiene `pausa_s` en cada uno.

    En las pausas la imagen sale nítida y detector_scan no la descarta por
    girar rápido. Al final se alinea con la orientación inicial, para no
    invalidar la geometría con la que se planificó el camino.
    """

    def __init__(self, pasos=8, pausa_s=0.6, velocidad=0.6, tolerancia=0.02,
                 timeout_s=40.0):
        self.pasos = pasos
        self.pausa_s = pausa_s
        self.velocidad = velocidad
        self.tolerancia = tolerancia
        self.timeout_s = timeout_s
        self.activa = False

    def iniciar(self, yaw, ahora):
        self.activa = True
        self.yaw_inicial = yaw
        self.paso = 1
        self.inicio = ahora
        self.fin_pausa = None

    def objetivo(self):
        return normalizar(self.yaw_inicial + 2.0 * math.pi * self.paso / self.pasos)

    def actualizar(self, yaw, ahora):
        """Devuelve la velocidad angular a ordenar; desactiva la mirada al terminar."""
        if not self.activa:
            return 0.0
        if ahora - self.inicio > self.timeout_s:
            self.activa = False
            return 0.0
        if self.fin_pausa is not None:
            if ahora < self.fin_pausa:
                return 0.0
            self.fin_pausa = None
            self.paso += 1
            if self.paso > self.pasos:
                self.activa = False
                return 0.0
        error = normalizar(self.objetivo() - yaw)
        if abs(error) <= self.tolerancia:
            self.fin_pausa = ahora + self.pausa_s
            return 0.0
        # Siempre antihorario mientras falte más de media vuelta del paso.
        if self.paso < self.pasos and error < -math.pi / 2:
            error += 2.0 * math.pi
        rapidez = min(self.velocidad, max(0.15, 1.5 * abs(error)))
        return math.copysign(rapidez, error)
