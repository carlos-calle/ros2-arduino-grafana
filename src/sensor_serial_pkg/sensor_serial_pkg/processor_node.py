# Archivo: processor_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

class ProcessorNode(Node):
    """
    Este nodo se suscribe a los datos crudos del sensor, los convierte a temperatura
    y los publica en un nuevo tópico.
    """
    def __init__(self):
        super().__init__('processor_node')
        
        # Suscriptor al tópico de datos crudos
        self.subscription = self.create_subscription(
            Int32,
            'sensor_data',
            self.listener_callback,
            10)
        
        # Publicador para el tópico de temperatura procesada
        self.publisher_ = self.create_publisher(Float32, 'temperature_celsius', 10)

    def listener_callback(self, msg):
        """
        Callback que se ejecuta cada vez que se recibe un mensaje con datos crudos.
        """
        raw_value = msg.data
        
        # Fórmula de conversión: mapea el rango 0-1023 a 0-100 °C.
        # Esta es una simulación; la fórmula cambiaría para un sensor real.
        temperature = (raw_value / 1023.0) * 100.0
        
        temp_msg = Float32()
        temp_msg.data = temperature
        
        self.publisher_.publish(temp_msg)
        self.get_logger().info(f'⚙️ Procesado: {raw_value} -> {temperature:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    node = ProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
