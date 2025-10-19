# Monitorización de Sensores en Tiempo Real con ROS 2, Docker e InfluxDB/Grafana

Sistema containerizado para adquirir datos en tiempo real desde **Arduino**, procesarlos con **ROS 2** y visualizarlos en **Grafana**. La arquitectura es desacoplada, reproducible y se levanta de forma automatizada con **Docker Compose** y un archivo de lanzamiento de ROS 2.

-----

## 🧩 Arquitectura

```
[Arduino] --/dev/ttyACM0--> [ROS 2: sensor_node] --> /sensor_data -->
                               [ROS 2: processor_node] --> /temperature_celsius -->
                                   [ROS 2: database_node] --> [InfluxDB] --> [Grafana]
```

Servicios orquestados por Docker Compose:

  - **ros2\_app** (`ros2_serial_app_reto`): Contenedor con los nodos de ROS 2 que se ejecutan automáticamente.
  - **influxdb**: Base de datos de series temporales para la telemetría.
  - **grafana**: Dashboard para la visualización en vivo.

-----

## 🚀 Guía de Inicio Rápido

### 0\) Requisitos para Windows + WSL2

  - **Docker Desktop** (con **WSL2 integration** activada para tu distro Ubuntu).
  - **Git**.
  - **Arduino** (con un potenciómetro en `A0`).
  - **WSL2 + usbipd-win**.

Instala y configura `usbipd-win` desde una terminal **PowerShell (Administrador)**:

```powershell
winget install usbipd
```

Conecta el Arduino y lista los dispositivos para encontrar su **BUSID**:

```powershell
usbipd list
```

Localiza el `BUSID` de tu Arduino (ej. `1-1`) y adjúntalo a WSL:

1.  Abre y mantén abierta una terminal de **Ubuntu (WSL)**.
2.  Desde **PowerShell (Administrador)**, ejecuta:
    ```powershell
    usbipd attach --wsl --busid 1-1
    ```
3.  Verifica que el dispositivo aparece en WSL:
    ```bash
    ls -l /dev/ttyACM*
    ```

> En Linux nativo no necesitas `usbipd`; solo asegúrate de que tu usuario pertenezca al grupo `dialout`.

-----

### 1\) Clonar el Repositorio

```bash
git clone https://github.com/carlos-calle/ros2-arduino-grafana.git
cd ros2-arduino-grafana
```

-----

### 2\) Levantar Todo el Stack

Con Docker Desktop en estado **Running**, ejecuta un único comando:

```bash
docker compose up -d --build
```

Este comando construirá la imagen, iniciará los tres contenedores y **ejecutará automáticamente los nodos de ROS 2** a través del archivo `monitor.launch.py`.

Para verificar que todos los servicios están corriendo:

```bash
docker ps
```

Deberías ver los tres contenedores: `influxdb`, `grafana` y `ros2_serial_app_reto`.

-----

### 3\) Verificar los Nodos de ROS 2

Los nodos ya están corriendo. La mejor forma de verificar su estado es viendo los logs en tiempo real:

```bash
docker compose logs -f ros2_app
```

Verás la salida de los tres nodos, incluyendo los mensajes del `database_node` confirmando que se están guardando datos en InfluxDB.

**Opcional: Interacción con ROS 2**
Si deseas inspeccionar los tópicos, puedes entrar al contenedor:

```bash
docker exec -it ros2_serial_app_reto bash
# Dentro, los tópicos ya están activos:
ros2 topic list
ros2 topic echo /temperature_celsius
```

-----

### 4\) Configurar y Ver Grafana

  - **Abre tu navegador**: [http://localhost:3000](https://www.google.com/search?q=http://localhost:3000)
  - **Login**: `admin` / `admin` (te pedirá cambiarla).

⚙️ **Configuration \> Data Sources \> Add data source \> InfluxDB**

  - **URL**: `http://influxdb:8086`
  - **Organization**: `ucuenca`
  - **Bucket**: `ros2_sensors`
  - **Token**: `my-super-secret-token`

Haz clic en **"Save & Test"** ✅.

Crea un dashboard y agrega un panel con la siguiente consulta **Flux**:

```flux
from(bucket: "ros2_sensors")
  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
  |> filter(fn: (r) => r["_measurement"] == "temperature_data")
  |> filter(fn: (r) => r["_field"] == "degrees_celsius")
```

Configura el rango en **Last 5 minutes** y el auto-refresco en **5s** para una visualización en vivo.

-----

## 🧹 Apagado y Limpieza

Para detener y eliminar todos los contenedores, redes y volúmenes:

```bash
docker compose down -v
```

-----

## 🛠️ Problemas Comunes

  - **Docker no funciona en WSL**: Activa *Settings → Resources → WSL Integration* en Docker Desktop y reinicia WSL (`wsl --shutdown`).
  - **Error de `usbipd` "No WSL 2 distribution running"**: Asegúrate de tener una terminal de WSL abierta antes de ejecutar `usbipd attach`.
  - **No se encuentra `/dev/ttyACM0`**: Desconecta y reconecta el Arduino, y vuelve a ejecutar `usbipd attach --wsl --busid <BUSID>`.
  - **InfluxDB tarda en estar disponible**: Revisa sus logs con `docker compose logs -f influxdb`. El `healthcheck` en `docker-compose.yml` debería manejar esto.

-----

## 📦 Versiones

| Componente | Versión |
| :--- | :--- |
| ROS 2 | Jazzy |
| InfluxDB | 2.7 |
| Grafana | latest |

-----

## 📄 Licencia

MIT
