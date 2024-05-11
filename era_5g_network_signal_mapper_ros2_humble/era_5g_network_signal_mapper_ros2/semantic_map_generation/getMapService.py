#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid

'''
Reference ROS node to get map server
'''

class Mapper(Node):
    def __init__(self):
        super().__init__('pcl2_to_costmap')

        # En self.response se almacenarán los datos del mapa de fila desde "/map_server/map"
        self.response = None

        # Creación de una solicitud para recibir datos del mapa desde "/map_server/map"
        client = self.create_client(GetMap, '/map_server/map')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio de map_server no disponible, esperando...')
        # Crear el mensaje de solicitud
        request = GetMap.Request()
        # Llamar al servicio y obtener la respuesta
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.response = future.result()
            # Procesar la respuesta aquí
            # En metadatos almacenamos información analizada del mapa recibido de "/map_server/map"

            self.get_logger().info('¡Datos del mapa recibidos del servidor de mapas!')
            # Puede acceder al mapa y los metadatos usando response.map y response.map_metadata
        else:
            self.get_logger().info('Error al recibir los datos del mapa.')


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
