# Patient Monitoring System Demo

This repository sketches a secure, MQTT-based patient monitoring stack:

- **ESP32 patient node** that reads vitals from MAX30102/MAX30100, DS18B20, and MPU6050 sensors, then publishes JSON payloads over MQTT with TLS.
- **Backend (FastAPI + MQTT bridge)** that subscribes to vitals, stores the latest readings, applies threshold rules, issues actuator commands (buzzer/LED) back to the device, and exposes REST endpoints for dashboards.

The code is intentionally lightweight so you can extend it with real sensors, a production database, and UI charts.

## Repository layout

- `firmware/esp32_patient_node.ino` – Arduino-style ESP32 firmware skeleton for the patient node.
- `backend/app/` – FastAPI application with MQTT bridge, rule evaluation, and in-memory store.
- `backend/requirements.txt` – Python dependencies for the backend service.
- `backend/.env.example` – Environment variables for MQTT and thresholds.

## Backend quickstart

1. Install Python dependencies:
   ```bash
   cd backend
   python -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```
2. Export MQTT/TLS settings (see `.env.example`). If you want to demo without a broker, you can still run the API and post simulated vitals.
3. Launch the API (this also starts the UDP discovery responder that ESP32 nodes use to find the backend on your LAN):
   ```bash
   uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
   ```
4. Exercise the API:
   - `GET /health` – readiness check.
   - `POST /simulate/vitals` – send a `VitalsMessage` JSON to test rule evaluation without MQTT.
   - `GET /vitals` and `GET /alerts` – list the latest readings and alert history.
   - `POST /device/<device_id>/vitals` and `POST /device/<device_id>/telemetry` – ESP32-facing endpoints used by the firmware to post vitals/telemetry when MQTT is not available.

When connected to a broker, the backend subscribes to `patients/+/vitals` and publishes commands such as `ALARM_ON` to `patients/<device_id>/cmd` when thresholds are exceeded.

### Local Wi-Fi demo with the ESP32 firmware

Use these steps to run the backend and have the ESP32 automatically discover it over the same Wi‑Fi network:

1. Start the backend (as above) and ensure UDP port 4211 is reachable on your machine. The discovery responder replies to `PMS_DISCOVER` broadcasts on that port and advertises `http://<backend-ip>:8000` by default. Override via `DISCOVERY_PORT`, `DISCOVERY_REQUEST`, `DISCOVERY_RESPONSE`, `DISCOVERY_SCHEME`, or `DISCOVERY_API_PORT` environment variables if needed.
2. Open `firmware/esp32_patient_node/esp32_patient_node.ino` in the Arduino IDE or `arduino-cli` and update the top of the file with your Wi‑Fi SSID/password, `DEVICE_ID`, and optional patient hash. Leave the discovery constants unless you changed them on the backend.
3. Select the ESP32 board in the IDE, connect the device over USB, and upload the sketch. On boot the ESP32 will:
   - Join your Wi‑Fi network.
   - Broadcast a UDP discovery request; the backend responds with its base URL.
   - POST vitals and telemetry every ~2 seconds to `/device/<DEVICE_ID>/vitals` and `/device/<DEVICE_ID>/telemetry`.
4. Watch the serial monitor for logs (discovery success, vitals payloads, backend responses). You can also call the backend directly with curl:
   ```bash
   curl http://<backend-ip>:8000/vitals
   curl http://<backend-ip>:8000/telemetry
   curl http://<backend-ip>:8000/alerts
   ```

## ESP32 firmware notes

- Replace Wi-Fi/MQTT credentials, the CA certificate, and the sensor stub functions (`readHeartRate`, `readSpO2`, `readTemperature`, `detectFall`) with real library calls.
- The firmware publishes to `patients/patient_node_01/vitals` every five seconds and listens for commands on `patients/patient_node_01/cmd` to toggle the buzzer/LED.
- Keep patient identifiers hashed in payloads to avoid storing PHI directly.

## Security considerations

- Use MQTT over TLS with a trusted CA (`MQTT_CA_CERT`) and strong per-device credentials.
- Configure broker ACLs so each patient node can only publish its own vitals and consume its own command topic.
- Add role-based authentication in whatever dashboard you build on top of the FastAPI endpoints.

## Next steps

- Persist vitals and alerts in a time-series database (InfluxDB, MongoDB, or Postgres + Timescale).
- Add JWT-protected dashboard routes and live charts (e.g., React + Chart.js) that query the FastAPI endpoints.
- Extend alert logic with debounce/hysteresis and configurable per-patient thresholds.
- Integrate real sensor libraries on the ESP32 and calibrate thresholds after bench testing.
