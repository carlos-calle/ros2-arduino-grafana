# Archivo: sensor_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time

class SensorNode(Node):
    """
    Este nodo lee datos de un puerto serial (Arduino), valida que sean numéricos,
    y los publica en el tópico 'sensor_data' como mensajes Int32.
    """
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_ = self.create_publisher(Int32, 'sensor_data', 10)
        
        # Bucle para intentar conectar al puerto serial de forma robusta
        while rclpy.ok():
            try:
                # Configura la conexión serial. Asegúrate que el puerto y baudios coinciden.
                self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1.0)
                self.get_logger().info('✅ Puerto serial /dev/ttyACM0 abierto correctamente.')
                break
            except serial.SerialException:
                self.get_logger().warn('⚠️ No se pudo abrir el puerto serial. Reintentando en 5 segundos...')
                time.sleep(5)
        
        # Crea un temporizador que llama a la función de publicación cada 0.4 segundos
        self.timer = self.create_timer(0.4, self.publish_data)

    def publish_data(self):
        """
        Lee una línea del puerto serial, la valida y la publica.
        """
        try:
            # Lee una línea y la decodifica/limpia
            line = self.serial_port.readline().decode('utf-8').strip()
            
            # Valida que la línea leída sea un número antes de procesarla
            if line.isdigit():
                value = int(line)
                msg = Int32()
                msg.data = value
                self.publisher_.publish(msg)
                self.get_logger().info(f'📦 Publicando dato crudo: {value}')
            elif line: # Si la línea no está vacía pero no es un dígito, advierte.
                self.get_logger().warn(f'Dato inválido recibido del serial: "{line}"')
                
        except serial.SerialException as e:
            self.get_logger().error(f'Error de comunicación serial: {e}')
        except Exception as e:
            self.get_logger().error(f'Error inesperado en el nodo sensor: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
