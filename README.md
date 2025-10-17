# Monitorización de Sensores en Tiempo Real con ROS 2, Docker e InfluxDB/Grafana

Sistema containerizado para adquirir datos en tiempo real desde **Arduino**, procesarlos con **ROS 2** y visualizarlos en **Grafana**. La arquitectura es desacoplada, reproducible y se levanta con **Docker Compose**.

---

## 🧩 Arquitectura

```
[Arduino] --/dev/ttyACM0--> [ROS 2: sensor_node] --> /sensor_data -->
                               [ROS 2: processor_node] --> /temperature_celsius -->
                                   [ROS 2: database_node] --> [InfluxDB] --> [Grafana]
```

Servicios:
- **ros2_app** (`ros2_serial_app_reto`): nodos ROS 2 (sensor/processor/database).
- **influxdb**: base de datos time-series (telemetría).
- **grafana**: panel en vivo.

---

## 🚀 Guía de Inicio Rápido

### 0) Requisitos para Windows + WSL2

- **Docker Desktop** (con **WSL2 integration** activada para tu distro Ubuntu)
- **Git**
- **Arduino** (potenciómetro en `A0`)
- **WSL2 + usbipd-win**

Instala y configura `usbipd-win`:

```powershell
winget install usbipd
```

Conecta el Arduino y lista dispositivos:

```powershell
usbipd list
```

Localiza el **BUSID** de tu Arduino (ej. `1-1`), luego:

1. Abre **Ubuntu (WSL)** y déjala **abierta**.
2. Adjunta el USB desde PowerShell (administrador):
   ```powershell
   usbipd attach --wsl --busid 1-1
   ```
3. Verifica en WSL:
   ```bash
   ls -l /dev/ttyACM* || echo "No se encontró /dev/ttyACM*"
   dmesg | tail -n 30
   ```

> En Linux nativo no necesitas `usbipd`; solo asegúrate de tener permisos de dialout para `/dev/ttyACM0`.

---

### 1) Clonar el repositorio

```bash
git clone https://github.com/carlos-calle/ros2-arduino-grafana.git
cd ros2-arduino-grafana
```

---

### 2) Levantar el stack

Asegúrate de que Docker Desktop esté **Running** y con WSL integrado.

```bash
docker compose up -d --build
docker ps
```

Verás:
- `influxdb` (healthy)
- `grafana` (puerto 3000)
- `ros2_serial_app_reto`

> Si tu dispositivo es `/dev/ttyUSB0`, modifica el `docker-compose.yml` en `devices:`.

---

### 3) Ejecutar los nodos de ROS 2

Entrar al contenedor:

```bash
docker exec -it ros2_serial_app_reto bash
```

Dentro:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Nodos en segundo plano
ros2 run sensor_serial_pkg sensor_node &
ros2 run sensor_serial_pkg processor_node &
ros2 run sensor_serial_pkg database_node &
```

Verifica tópicos:

```bash
ros2 topic list
ros2 topic echo /sensor_data
```

---

### 4) Grafana (visualización)

- Abre: [http://localhost:3000](http://localhost:3000)
- Login: `admin` / `admin` (cámbiala)

⚙️ **Configuration > Data Sources > Add data source > InfluxDB**

- URL: `http://influxdb:8086`
- Org: `ucuenca`
- Bucket: `ros2_sensors`
- Token: `my-super-secret-token`

"Save & Test" ✅

Crea un dashboard y agrega un panel con Flux:

```flux
from(bucket: "ros2_sensors")
  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
  |> filter(fn: (r) => r["_measurement"] == "temperature_data")
  |> filter(fn: (r) => r["_field"] == "degrees_celsius")
```

Rango: **Last 5 minutes** | Auto-refresh: **5s**

---

## 🧹 Apagado y limpieza

```bash
docker compose down -v
```

---

## 🛠️ Problemas comunes

- **Docker no funciona en WSL**
  - Activa Settings → Resources → **WSL Integration**
  - Reinicia WSL:
    ```powershell
    wsl --shutdown
    ```

- **usbipd error: "There is no WSL 2 distribution running"**
  - Mantén una terminal WSL abierta antes de ejecutar `usbipd attach`.

- **No existe `/dev/ttyACM0`**
  - Reconecta Arduino y:
    ```powershell
    usbipd attach --wsl --busid <BUSID>
    ```
  - En WSL:
    ```bash
    dmesg | tail -n 50
    ```

- **InfluxDB tarda en estar disponible**
  ```bash
  docker compose logs -f influxdb
  ```

- **Permisos de serie**
  Asegúrate de que en el servicio ROS 2 existe:
  ```yaml
  devices:
    - "/dev/ttyACM0:/dev/ttyACM0"
  group_add:
    - dialout
  ```

---

## 📦 Versiones

| Componente | Versión |
|------------|---------|
| ROS 2      | Jazzy   |
| InfluxDB   | 2.7     |
| Grafana    | latest  |

---

## 📄 Licencia

MIT

---
