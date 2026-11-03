"""Deja activos los controladores de ros2_control aunque Gazebo tarde en arrancar."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def proximo_paso(estado):
    """
    Pedido para el controller_manager según el estado de un controlador.

    None si no hace falta nada (activo o en un estado que no se toca).
    """
    if estado is None:
        return 'cargar'
    if estado == 'unconfigured':
        return 'configurar'
    if estado == 'inactive':
        return 'activar'
    return None


class ActivarControladores(Node):
    """
    Carga, configura y activa los controladores con esperas largas.

    El spawner del simulador se rinde a los 30 s, pero al arrancar Gazebo
    puede quedar hasta un minuto inicializando los sensores con render y el
    controller_manager no contesta. Sin joint_state_broadcaster no hay
    /joint_states ni /odom y el robot no se mueve. Este nodo reintenta con
    paciencia y termina cuando todos quedan activos.
    """

    def __init__(self):
        super().__init__('activar_controladores')
        from controller_manager_msgs.srv import (ConfigureController, ListControllers,
                                                 LoadController, SwitchController)
        self.SwitchController = SwitchController
        self.declare_parameter('controladores', ['joint_state_broadcaster'])
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('timeout_llamada_s', 90.0)
        self.declare_parameter('timeout_total_s', 300.0)
        self.controladores = list(self.get_parameter('controladores').value)
        self.timeout_llamada_s = self.get_parameter('timeout_llamada_s').value
        self.timeout_total_s = self.get_parameter('timeout_total_s').value
        cm = self.get_parameter('controller_manager').value
        self.listar = self.create_client(ListControllers, f'{cm}/list_controllers')
        self.cargar = self.create_client(LoadController, f'{cm}/load_controller')
        self.configurar = self.create_client(ConfigureController, f'{cm}/configure_controller')
        self.cambiar = self.create_client(SwitchController, f'{cm}/switch_controller')

    def llamar(self, cliente, pedido):
        """Respuesta del servicio, o None si no contestó a tiempo."""
        if not cliente.wait_for_service(timeout_sec=self.timeout_llamada_s):
            return None
        futuro = cliente.call_async(pedido)
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=self.timeout_llamada_s)
        return futuro.result() if futuro.done() else None

    def estados(self):
        respuesta = self.llamar(self.listar, self.listar.srv_type.Request())
        if respuesta is None:
            return None
        return {c.name: c.state for c in respuesta.controller}

    def avanzar(self, nombre, paso):
        if paso == 'cargar':
            pedido = self.cargar.srv_type.Request(name=nombre)
            return self.llamar(self.cargar, pedido)
        if paso == 'configurar':
            pedido = self.configurar.srv_type.Request(name=nombre)
            return self.llamar(self.configurar, pedido)
        pedido = self.SwitchController.Request(
            activate_controllers=[nombre], activate_asap=True,
            strictness=self.SwitchController.Request.BEST_EFFORT)
        pedido.timeout.sec = int(self.timeout_llamada_s)
        return self.llamar(self.cambiar, pedido)

    def ejecutar(self):
        """Devuelve True cuando todos los controladores quedaron activos."""
        inicio = self.get_clock().now()
        reintentos = 0
        while rclpy.ok():
            if (self.get_clock().now() - inicio).nanoseconds / 1e9 > self.timeout_total_s:
                self.get_logger().error(
                    f'Controladores sin activar después de {self.timeout_total_s:.0f} s.')
                return False
            estados = self.estados()
            if estados is None:
                self.get_logger().info('Esperando al controller_manager...')
                continue
            pasos = {n: proximo_paso(estados.get(n)) for n in self.controladores}
            pendientes = {n: p for n, p in pasos.items() if p is not None}
            if not pendientes:
                if reintentos:
                    self.get_logger().info(
                        f'Controladores activos: {", ".join(self.controladores)}.')
                return True
            reintentos += 1
            for nombre, paso in pendientes.items():
                self.get_logger().warn(f'{nombre}: {estados.get(nombre, "sin cargar")}, '
                                       f'pido {paso}.')
                self.avanzar(nombre, paso)
            # Dar tiempo al spawner del simulador antes de volver a preguntar.
            rclpy.spin_once(self, timeout_sec=2.0)
        return False


def main(args=None):
    rclpy.init(args=args)
    nodo = ActivarControladores()
    try:
        nodo.ejecutar()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
