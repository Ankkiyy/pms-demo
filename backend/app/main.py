"""FastAPI application for the central monitoring unit."""
from __future__ import annotations

from fastapi import FastAPI, HTTPException

from .mqtt_client import bridge
from .discovery import discovery_responder
from .rules import evaluate_alert
from .schemas import Alert, Command, TelemetryMessage, Vitals, VitalsMessage
from .store import data_store

app = FastAPI(title="Patient Monitoring System", version="0.1.0")


@app.on_event("startup")
async def startup_event() -> None:
    bridge.start()
    discovery_responder.start()


@app.on_event("shutdown")
async def shutdown_event() -> None:
    bridge.stop()
    discovery_responder.stop()


@app.get("/health")
async def health() -> dict[str, str]:
    return {"status": "ok"}


@app.get("/vitals", response_model=list[VitalsMessage])
async def get_all_vitals() -> list[VitalsMessage]:
    return data_store.get_latest_vitals()


@app.get("/vitals/{device_id}", response_model=VitalsMessage)
async def get_vitals(device_id: str) -> VitalsMessage:
    vitals = data_store.get_vitals_for_device(device_id)
    if not vitals:
        raise HTTPException(status_code=404, detail="Device not found")
    return vitals


@app.get("/alerts", response_model=list[Alert])
async def get_alerts() -> list[Alert]:
    return data_store.get_alerts()


@app.get("/telemetry", response_model=list[TelemetryMessage])
async def get_telemetry() -> list[TelemetryMessage]:
    return data_store.get_latest_telemetry()


@app.post("/simulate/vitals", response_model=dict)
async def simulate_vitals(message: VitalsMessage) -> dict:
    """Allow posting test vitals without an MQTT broker (useful for demos)."""
    data_store.update_vitals(message)
    reason = evaluate_alert(message.vitals)
    if reason:
        data_store.record_alert(
            Alert(
                device_id=message.device_id,
                reason=reason,
                vitals=message.vitals,
                timestamp=message.timestamp,
            )
        )
        # Mirror what would be sent over MQTT for visibility in the response
        command = Command(cmd="ALARM_ON", reason=reason)
        return {"alert": reason, "command": command.dict()}
    return {"alert": None}


@app.post("/device/{device_id}/vitals", response_model=dict)
async def ingest_vitals(device_id: str, message: VitalsMessage) -> dict:
    """Device-facing vitals ingestion endpoint for ESP32 nodes."""

    if message.device_id != device_id:
        raise HTTPException(status_code=400, detail="Device ID mismatch")

    data_store.update_vitals(message)
    reason = evaluate_alert(message.vitals)
    if reason:
        alert = Alert(
            device_id=message.device_id,
            reason=reason,
            vitals=message.vitals,
            timestamp=message.timestamp,
        )
        data_store.record_alert(alert)
        cmd = Command(cmd="ALARM_ON", reason=reason)
        return {"alert": reason, "command": cmd.dict()}
    return {"alert": None}


@app.post("/device/{device_id}/telemetry", response_model=TelemetryMessage)
async def ingest_telemetry(device_id: str, payload: dict) -> TelemetryMessage:
    """Accept arbitrary JSON from the ESP32 for debugging or extra metrics."""

    message = TelemetryMessage(device_id=device_id, data=payload)
    data_store.record_telemetry(message)
    return message

