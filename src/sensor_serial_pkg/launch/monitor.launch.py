# src/sensor_serial_pkg/launch/monitor.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    """Lanza los tres nodos del sistema de monitorización."""

    # Nodo que lee del puerto serial
    sensor_node = Node(
        package='sensor_serial_pkg',
        executable='sensor_node',
        name='sensor_node',
        output='screen'
    )

    # Nodo que procesa los datos crudos a temperatura
    processor_node = Node(
        package='sensor_serial_pkg',
        executable='processor_node',
        name='processor_node',
        output='screen'
    )

    # Nodo que escribe en la base de datos
    database_node = Node(
        package='sensor_serial_pkg',
        executable='database_node',
        name='database_node',
        output='screen'
    )

    return LaunchDescription([
        sensor_node,
        processor_node,
        
        # Agregamos un retraso de 5 segundos antes de lanzar el database_node
        # para dar tiempo a que InfluxDB se inicie completamente.
        TimerAction(
            period=5.0,
            actions=[database_node]
        )
    ])
