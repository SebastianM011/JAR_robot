"""Pruebas del giro de 360° por pasos sin ejecutar ROS."""

import math

from jar_exploracion.mirada import MiradaPorPasos, giro_seguro, normalizar, radio_de_giro

DT = 0.1


def simular(mirada, yaw, ahora=0.0, maximo_s=60.0):
    """Integra la velocidad ordenada como un robot ideal; devuelve el recorrido."""
    girado = 0.0
    pausas = 0
    quieto_antes = False
    while mirada.activa and ahora < maximo_s:
        w = mirada.actualizar(yaw, ahora)
        quieto = w == 0.0 and mirada.activa
        pausas += quieto and not quieto_antes
        quieto_antes = quieto
        yaw = normalizar(yaw + w * DT)
        girado += w * DT
        ahora += DT
    return yaw, girado, pausas, ahora


def test_vuelta_completa_con_pausas_y_alineada():
    mirada = MiradaPorPasos(pasos=8, pausa_s=0.6, velocidad=0.6)
    mirada.iniciar(1.0, 0.0)
    yaw, girado, pausas, duracion = simular(mirada, 1.0)
    assert not mirada.activa
    assert abs(girado - 2 * math.pi) < 0.1
    assert abs(normalizar(yaw - 1.0)) <= 0.02
    assert pausas == 8
    assert duracion < 25.0


def test_cruza_pi_sin_volver_atras():
    """Arrancando cerca de pi, cada paso sigue en sentido antihorario."""
    mirada = MiradaPorPasos(pasos=4, pausa_s=0.2, velocidad=0.6)
    mirada.iniciar(3.0, 0.0)
    _, girado, _, _ = simular(mirada, 3.0)
    assert abs(girado - 2 * math.pi) < 0.1


def test_timeout_desactiva():
    mirada = MiradaPorPasos(timeout_s=1.0)
    mirada.iniciar(0.0, 0.0)
    # El robot no gira (por ejemplo, trabado): la mirada se abandona.
    for i in range(30):
        mirada.actualizar(0.0, i * DT)
    assert not mirada.activa
    assert mirada.actualizar(0.0, 5.0) == 0.0


def test_giro_seguro():
    radio = radio_de_giro(0.37, 0.24, 0.05)
    assert math.isclose(radio, math.hypot(0.185, 0.12) + 0.05)
    assert giro_seguro([(1.0, 0.0)], 0.5, radio)
    assert not giro_seguro([(0.2, 0.1)], 0.5, radio)   # algo dentro del círculo
    assert not giro_seguro([], 0.2, radio)              # pasillo angosto
    assert not giro_seguro([], None, radio)             # fuera del mapa
    assert giro_seguro([], 0.5, radio)
