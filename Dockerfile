# -- Etapa 1: Entorno Base y Dependencias --
# Usamos la imagen oficial de ROS 2 Jazzy como punto de partida.
FROM osrf/ros:jazzy-desktop

# Para evitar preguntas interactivas durante la instalación de paquetes.
ENV DEBIAN_FRONTEND=noninteractive

# Instala las dependencias del sistema:
# - colcon: El compilador de ROS 2.
# - python3-serial: La librería para comunicarse con el Arduino.
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-serial \
    python3-pip \
    && pip3 install influxdb-client --break-system-packages \
    && rm -rf /var/lib/apt/lists/*

# -- Etapa 2: Copia y Compilación del Código --
# Establece el directorio de trabajo principal dentro del contenedor.
WORKDIR /root/ros2_serial_ws

# Copia tu código fuente desde tu PC al directorio 'src' del workspace en la imagen.
COPY ./src ./src

# "Source" el entorno de ROS 2 y luego compila tu workspace.
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# -- Etapa 3: Configuración del Punto de Entrada --
# Este script se ejecutará cada vez que inicies el contenedor.
# Su función es activar los entornos de ROS y de tu workspace compilado.
COPY <<'EOF' /docker-entrypoint.sh
#!/bin/bash
set -e
# Activa el entorno de ROS 2
source "/opt/ros/jazzy/setup.bash"
# Activa el entorno de tu workspace
source "/root/ros2_serial_ws/install/setup.bash"
# Ejecuta cualquier comando que le pases al 'docker run'
exec "$@"
EOF

# Da permisos de ejecución al script.
RUN chmod +x /docker-entrypoint.sh

# Establece nuestro script como el punto de entrada.
ENTRYPOINT ["/docker-entrypoint.sh"]

# Si no se especifica ningún comando, por defecto abrirá una terminal bash.
CMD ["bash"]
