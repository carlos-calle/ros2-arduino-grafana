import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

class DatabaseNode(Node):
    def __init__(self):
        super().__init__('database_node')
        self.subscription = self.create_subscription(
            Float32,
            'temperature_celsius',
            self.listener_callback,
            10)

        # --- Configuración de InfluxDB ---
        self.influx_url = "http://influxdb:8086" # Usamos el nombre del contenedor
        self.influx_token = "my-super-secret-token"
        self.influx_org = "ucuenca"
        self.influx_bucket = "ros2_sensors"

        self.client = InfluxDBClient(url=self.influx_url, token=self.influx_token, org=self.influx_org)
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
        self.get_logger().info("✅ Conectado a InfluxDB.")

    def listener_callback(self, msg):
        temp_value = msg.data
        point = Point("temperature_data") \
            .tag("sensor_id", "potentiometer_A0") \
            .field("degrees_celsius", temp_value)

        self.write_api.write(bucket=self.influx_bucket, org=self.influx_org, record=point)
        self.get_logger().info(f"💾 Guardado en DB: {temp_value:.2f} °C")

def main(args=None):
    rclpy.init(args=args)
    node = DatabaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
