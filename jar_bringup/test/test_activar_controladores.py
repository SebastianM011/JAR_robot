"""Secuencia de pedidos al controller_manager según el estado del controlador."""

from jar_bringup.activar_controladores import proximo_paso


def test_proximo_paso():
    assert proximo_paso(None) == 'cargar'
    assert proximo_paso('unconfigured') == 'configurar'
    assert proximo_paso('inactive') == 'activar'
    assert proximo_paso('active') is None
    assert proximo_paso('finalized') is None
