"""Pruebas del agrupamiento de víctimas sin ejecutar ROS."""

import math

from jar_mision.victimas import AgrupadorVictimas, centro_desde_cara


def test_centro_se_aleja_del_robot():
    x, y = centro_desde_cara((0.0, 0.0), (1.0, 0.0), 0.09)
    assert math.isclose(x, 1.09) and math.isclose(y, 0.0)


def test_centro_sobre_el_robot():
    assert centro_desde_cara((1.0, 1.0), (1.0, 1.0), 0.09) is None


def test_publica_una_vez_al_juntar_minimas():
    agrupador = AgrupadorVictimas(victimas=2, detecciones_minimas=3, radio_grupo=0.3)
    assert agrupador.agregar(1.0, 1.0) is None
    assert agrupador.agregar(1.1, 1.0) is None
    assert agrupador.agregar(1.0, 1.1) == (1.0, 1.0)
    assert agrupador.agregar(1.0, 1.0) is None
    assert agrupador.publicados == 1


def test_mediana_ignora_detecciones_lejanas():
    agrupador = AgrupadorVictimas(victimas=1, detecciones_minimas=4, radio_grupo=0.5)
    for punto in [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]:
        agrupador.agregar(*punto)
    assert agrupador.agregar(0.4, 0.0) == (0.0, 0.0)


def test_no_supera_la_cantidad_de_victimas():
    agrupador = AgrupadorVictimas(victimas=1, detecciones_minimas=1, radio_grupo=0.3)
    assert agrupador.agregar(0.0, 0.0) == (0.0, 0.0)
    assert agrupador.agregar(2.0, 2.0) is None
    assert len(agrupador.grupos) == 2
