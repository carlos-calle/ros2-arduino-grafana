# Archivo: monitor_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MonitorNode(Node):
    """
    Este nodo se suscribe al tópico de temperatura procesada y muestra
    los valores en la consola.
    """
    def __init__(self):
        super().__init__('monitor_node')
        
        # Suscriptor al tópico de temperatura en Celsius
        self.subscription = self.create_subscription(
            Float32,
            'temperature_celsius',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        """
        Callback que se ejecuta al recibir un mensaje de temperatura.
        """
        # Muestra la temperatura recibida formateada con dos decimales
        self.get_logger().info(f'🌡️ Temperatura actual: {msg.data:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
