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
3. Launch the API:
   ```bash
   uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
   ```
4. Exercise the API:
   - `GET /health` – readiness check.
   - `POST /simulate/vitals` – send a `VitalsMessage` JSON to test rule evaluation without MQTT.
   - `GET /vitals` and `GET /alerts` – list the latest readings and alert history.

When connected to a broker, the backend subscribes to `patients/+/vitals` and publishes commands such as `ALARM_ON` to `patients/<device_id>/cmd` when thresholds are exceeded.

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
