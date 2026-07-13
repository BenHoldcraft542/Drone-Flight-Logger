# Drone Flight Logger

Logs real GPS telemetry from a bare-bones ESP32 + GPS module, stores it in
SQLite, and displays flights in a 2D map + 3D view web app.

## How data flows

1. **ESP32** reads GPS while flying and appends each fix to a file on its
   own flash storage (LittleFS) -- no WiFi needed in the field.
2. Next time it powers on **near WiFi**, it uploads that file to your
   computer and clears it, then starts a fresh log for the next flight.
3. The **backend** (FastAPI) receives the upload, reconstructs real
   timestamps, and stores everything in **SQLite**.
4. The **frontend** fetches flights from the backend and renders the path
   on a 2D map and in a 3D scene.

## 1. Wire up the ESP32

NEO-6M (or similar) GPS module -> ESP32, via UART2:

| GPS module | ESP32     |
|------------|-----------|
| TX         | GPIO16 (RX2) |
| RX         | GPIO17 (TX2) |
| VCC        | 3.3V (check your module) |
| GND        | GND       |

## 2. Flash the firmware

- Open `firmware/drone_logger.ino` in the Arduino IDE (with ESP32 board
  support installed).
- Install libraries: **TinyGPSPlus**, **ArduinoJson** (Library Manager).
- Edit the CONFIG section at the top:
  - `WIFI_SSID` / `WIFI_PASSWORD` -- the network your logging computer is on
  - `SERVER_URL` -- `http://<your-computer's-LAN-IP>:8000/api/ingest/flight`
  - `DRONE_NAME` -- whatever you want flights tagged with
- Flash it. The drone can now fly untethered; bring it back within WiFi
  range and power-cycle it to trigger the upload.

## 3. Run the backend

```bash
cd backend
pip install -r requirements.txt
uvicorn app:app --reload --host 0.0.0.0 --port 8000
```

Find your computer's LAN IP (so the ESP32 can reach it) with `ipconfig`
(Windows) or `ifconfig` / `ip a` (Mac/Linux), and make sure it matches
`SERVER_URL` in the firmware. Your firewall must allow inbound connections
on port 8000.

The database file is created automatically at `database/flights.db` the
first time a flight is ingested.

## 4. Open the web app

Visit **http://localhost:8000** (or `http://<computer-IP>:8000` from
another device). Pick a flight from the dropdown to see its path in 2D and
3D, plus altitude/speed/GPS-quality readouts.

## Optional: real flight-controller telemetry

If you ever upgrade to a flight controller that speaks MAVLink
(ArduPilot/PX4), `ingest/mavlink_logger.py` logs live telemetry directly
into the same database -- no firmware needed, just run it on a computer
connected to the telemetry radio.

## Adding an IMU later

The database and ingest endpoint already have `roll` / `pitch` / `yaw`
columns. Once you wire up an IMU (e.g. MPU6050) to the ESP32, add those
values to the JSON object in `writeFix()` in the firmware -- everything
downstream (backend, frontend) already knows what to do with them.
