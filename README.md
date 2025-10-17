# Monitorización de Sensores en Tiempo Real con ROS 2, Docker y Grafana

Este repositorio contiene un sistema completo y containerizado para adquirir datos en tiempo real de un sensor físico (Arduino), procesarlos a través de un pipeline de ROS 2 y visualizarlos en un dashboard de Grafana en vivo. El proyecto demuestra una arquitectura robusta y escalable para aplicaciones modernas de robótica e IoT, orquestada completamente con Docker Compose.

-----

## Características Principales

  * **Integración con Hardware**: Se conecta directamente con un Arduino a través del puerto serie USB para capturar datos de un sensor del mundo real.
  * **Pipeline de ROS 2 Multi-etapa**: Implementa un sistema modular de múltiples nodos en ROS 2 para la ingesta de datos crudos, su procesamiento y el registro en la base de datos.
  * **Base de Datos de Series Temporales**: Utiliza **InfluxDB** como una base de datos de alto rendimiento para almacenar telemetría de sensores, ideal para cargas de trabajo de IoT y monitorización.
  * **Visualización de Datos en Vivo**: Cuenta con un dashboard de **Grafana** para el ploteo y análisis en tiempo real de las lecturas del sensor.
  * **Completamente Containerizado**: Todo el stack (ROS 2, InfluxDB, Grafana) se gestiona a través de **Docker Compose**, asegurando una configuración reproducible y portátil que se levanta con un solo comando.

-----

## Arquitectura del Sistema

El proyecto sigue una arquitectura desacoplada y orientada a servicios. El flujo de datos va desde el hardware físico hasta la capa de visualización a través de una serie de servicios especializados, conectados por una red de Docker dedicada.

1.  **Arduino**: Lee un valor analógico de un potenciómetro y lo escribe en el puerto serie.
2.  **App de ROS 2 (`ros2_app`)**:
      * `sensor_node`: Se conecta al puerto serie del Arduino (`/dev/ttyACM0`) y publica el valor entero crudo en el tópico `/sensor_data`.
      * `processor_node`: Se suscribe a `/sensor_data`, convierte el valor crudo a grados Celsius y publica el resultado en el tópico `/temperature_celsius`.
      * `database_node`: Se suscribe a `/temperature_celsius` y escribe cada punto de datos en la base de datos InfluxDB.
3.  **InfluxDB (`influxdb`)**: Una base de datos de series temporales que recibe y almacena los datos de temperatura.
4.  **Grafana (`grafana`)**: Una plataforma de visualización que consulta InfluxDB y muestra los datos en un dashboard en vivo.

-----

## 🚀 Guía de Inicio Rápido

### Prerrequisitos

  * **Docker** y **Docker Compose** instalados.
  * Una placa **Arduino** con un potenciómetro conectado al pin `A0`.
  * (Para usuarios de Windows) **WSL2** con `usbipd-win` instalado y configurado para compartir el puerto USB del Arduino.

### Instalación y Ejecución

1.  **Clonar el Repositorio**

    ```bash
    git clone https://github.com/carlos-calle/ros2-arduino-grafana.git
    cd ros2-arduino-grafana
    ```

2.  **Lanzar el Sistema**

    Asegúrate de que tu Arduino esté conectado. Luego, desde la raíz del proyecto, ejecuta:

    ```bash
    docker compose up -d --build
    ```

    Este comando hará lo siguiente:

      * Construirá la imagen personalizada de ROS 2 desde el `Dockerfile`.
      * Descargará las imágenes oficiales de InfluxDB y Grafana.
      * Creará una red y volúmenes de datos persistentes.
      * Lanzará los tres servicios, esperando a que InfluxDB esté saludable antes de iniciar la app de ROS 2.

4.  **Ejecutar los Nodos de ROS 2**
    Abre una nueva terminal y ejecuta lo siguiente para entrar al contenedor de ROS 2 en ejecución:

    ```bash
    docker exec -it ros2_serial_app_reto bash
    ```

    Una vez dentro del contenedor, lanza los nodos:

    ```bash
    # Activa los entornos de ROS 2 y del workspace
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

    # Ejecuta los nodos en segundo plano
    ros2 run sensor_serial_pkg sensor_node &
    ros2 run sensor_serial_pkg processor_node &
    ros2 run sensor_serial_pkg database_node &
    ```

    Deberías ver en la consola los logs confirmando que los datos se están publicando, procesando y guardando en la base de datos.

-----

## 📈 Uso: El Dashboard de Grafana

Con el sistema en funcionamiento, ya puedes visualizar los datos.

1.  **Accede a Grafana**: Abre tu navegador web y navega a `http://localhost:3000`.
2.  **Inicia Sesión**: Usa las credenciales por defecto (`admin` / `admin`) y establece una nueva contraseña.
3.  **Configura la Fuente de Datos**:
      * Navega a **Configuration (⚙️) \> Data Sources**.
      * Haz clic en **Add data source** y selecciona **InfluxDB**.
      * Establece la **URL** como `http://influxdb:8086`.
      * En **InfluxDB Details**, proporciona las credenciales del archivo `docker-compose.yml` (`ucuenca`, `my-super-secret-token`, etc.).
      * Haz clic en **Save & test**.
4.  **Crea un Panel**:
      * Navega a **Dashboards (+)** y crea un nuevo dashboard.
      * Añade una visualización y selecciona tu fuente de datos InfluxDB.
      * Usa el editor de consultas **Flux** o pega el siguiente código para mostrar los datos de temperatura:
        ```js
        from(bucket: "ros2_sensors")
          |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
          |> filter(fn: (r) => r["_measurement"] == "temperature_data")
          |> filter(fn: (r) => r["_field"] == "degrees_celsius")
        ```
      * Configura el rango de tiempo en **"Last 5 minutes"** y activa el auto-refresco para una vista en tiempo real.

-----

## 🧹 Apagado del Sistema

Para detener y eliminar todos los contenedores, redes y volúmenes, ejecuta el siguiente comando desde la raíz del proyecto:

```bash
docker compose down -v
```
