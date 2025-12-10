"""FastAPI application for the central monitoring unit."""
from __future__ import annotations

import asyncio
import logging
import os
import socket
from datetime import datetime
from random import random
from typing import Iterable

import httpx

from fastapi import Body, FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse

from .config import discovery_config, thresholds
from .mqtt_client import bridge
from .discovery import discovery_responder
from .rules import evaluate_alert
from .schemas import Alert, Command, TelemetryMessage, Vitals, VitalsMessage
from .store import data_store

app = FastAPI(title="Patient Monitoring System", version="0.1.0")

_cors_origins = os.getenv(
    "CORS_ALLOW_ORIGINS",
    "http://localhost:8080,http://127.0.0.1:8080",
)
_allowed_origins = [origin.strip() for origin in _cors_origins.split(",") if origin.strip()]

app.add_middleware(
    CORSMiddleware,
    allow_origins=_allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

logger = logging.getLogger(__name__)

OLLAMA_MODEL = os.getenv("OLLAMA_MODEL", "deepseek-r1:1.5b")
OLLAMA_ENDPOINT = os.getenv("OLLAMA_ENDPOINT", "http://localhost:11434/api/chat")
ENABLE_DEMO_PATIENTS = os.getenv("ENABLE_DEMO_PATIENTS", "true").lower() == "true"

_demo_task: asyncio.Task | None = None
_DEMO_PATIENTS = [
    {
        "device_id": "esp-01",
        "patient_hash": "alice-vasquez",
        "heart_rate": 76.0,
        "spo2": 98.0,
        "temperature": 36.6,
        "ambient_temperature": 23.4,
        "humidity": 42.0,
        "activity_level": 0.18,
        "activity_state": "sitting",
        "steps_walked": 1240,
    },
    {
        "device_id": "esp-02",
        "patient_hash": "brandon-cho",
        "heart_rate": 88.0,
        "spo2": 94.0,
        "temperature": 37.1,
        "ambient_temperature": 24.1,
        "humidity": 48.0,
        "activity_level": 0.32,
        "activity_state": "walking",
        "steps_walked": 2530,
    },
    {
        "device_id": "esp-03",
        "patient_hash": "carol-mensah",
        "heart_rate": 64.0,
        "spo2": 97.0,
        "temperature": 36.4,
        "ambient_temperature": 22.8,
        "humidity": 38.0,
        "activity_level": 0.12,
        "activity_state": "sleeping",
        "steps_walked": 840,
    },
]


@app.on_event("startup")
async def startup_event() -> None:
    bridge.start()
    discovery_responder.start()
    data_store.set_event_loop(asyncio.get_running_loop())
    if ENABLE_DEMO_PATIENTS:
        _start_demo_patients()
    host_ip = _local_ip()
    logger.info(
        "Backend reachable on http://%s:%s (docs at http://%s:%s/docs)",
        host_ip,
        discovery_config.advertised_port,
        host_ip,
        discovery_config.advertised_port,
    )


def _local_ip() -> str:
    """Best-effort lookup of the primary LAN IPv4 address."""

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"

@app.on_event("shutdown")
async def shutdown_event() -> None:
    bridge.stop()
    discovery_responder.stop()
    _stop_demo_patients()


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


@app.get("/stream/vitals")
async def stream_vitals() -> StreamingResponse:
    """Server-sent events stream of the latest vitals per device."""

    queue: asyncio.Queue[VitalsMessage] = asyncio.Queue()
    data_store.set_event_loop(asyncio.get_running_loop())
    data_store.register_listener(queue)

    async def event_generator() -> Iterable[str]:
        try:
            for message in data_store.get_latest_vitals():
                yield f"data: {message.json()}\n\n"

            while True:
                message = await queue.get()
                yield f"data: {message.json()}\n\n"
        except asyncio.CancelledError:
            raise
        finally:
            data_store.unregister_listener(queue)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/ai/ask")
async def ask_assistant(
    payload: dict = Body(..., example={"question": "How is patient A?", "device_id": "demo"})
) -> dict[str, str]:
    """Answer questions with quick rules or Ollama agentic reasoning."""

    question = str(payload.get("question", "")).strip()
    device_id = payload.get("device_id")
    provider = str(payload.get("provider", "")).lower()
    use_ollama = provider == "ollama" or bool(payload.get("use_ollama"))

    if not question:
        raise HTTPException(status_code=400, detail="Question is required")

    target_vitals = None
    if device_id:
        target_vitals = data_store.get_vitals_for_device(device_id)
        if not target_vitals:
            raise HTTPException(status_code=404, detail="Device not found")

    if use_ollama:
        answer = await _agentic_patient_scan(question, device_id)
    elif target_vitals:
        answer = _build_patient_response(question, target_vitals)
    else:
        vitals_list = data_store.get_latest_vitals()
        if not vitals_list:
            return {"answer": "No vitals available yet."}

        flagged = [v for v in vitals_list if evaluate_alert(v.vitals)]
        summaries = ", ".join(f"{v.device_id} ({v.patient_hash[:8]})" for v in flagged)
        if flagged:
            headline = (
                f"{len(flagged)} patient(s) need attention: {summaries}. "
                "Others are within configured thresholds."
            )
        else:
            headline = "All monitored patients are within the configured thresholds."

        answer = headline

    return {"answer": answer}


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


def _start_demo_patients() -> None:
    """Kick off a background task that streams demo vitals."""

    global _demo_task
    if _demo_task and not _demo_task.done():
        return

    loop = asyncio.get_running_loop()
    _demo_task = loop.create_task(_run_demo_patients())


def _stop_demo_patients() -> None:
    global _demo_task
    if _demo_task and not _demo_task.done():
        _demo_task.cancel()
    _demo_task = None


async def _run_demo_patients() -> None:
    """Continuously emit realistic demo vitals for three patients."""

    logger.info("Starting demo patient vitals stream")
    try:
        while True:
            for patient in _DEMO_PATIENTS:
                vitals = _next_vitals(patient)
                message = VitalsMessage(
                    device_id=patient["device_id"],
                    patient_hash=patient["patient_hash"],
                    timestamp=datetime.utcnow(),
                    vitals=vitals,
                )
                data_store.update_vitals(message)
                reason = evaluate_alert(vitals)
                if reason:
                    data_store.record_alert(
                        Alert(
                            device_id=message.device_id,
                            reason=reason,
                            vitals=vitals,
                            timestamp=message.timestamp,
                        )
                    )
                await asyncio.sleep(1.5)
            await asyncio.sleep(0.5)
    except asyncio.CancelledError:
        logger.info("Demo patient task cancelled")
        raise


def _next_vitals(patient: dict) -> Vitals:
    """Generate lightly jittered vitals with occasional alerts."""

    heart_rate = _bounded_jitter(patient["heart_rate"], variance=6.0, minimum=52, maximum=140)
    spo2 = _bounded_jitter(patient["spo2"], variance=1.5, minimum=88, maximum=100)
    temperature = _bounded_jitter(
        patient["temperature"], variance=0.3, minimum=35.6, maximum=39.4
    )
    ambient_temperature = _bounded_jitter(
        patient["ambient_temperature"], variance=0.6, minimum=19.0, maximum=28.0
    )
    humidity = _bounded_jitter(patient["humidity"], variance=5.0, minimum=25.0, maximum=70.0)
    activity_level = _bounded_jitter(
        patient["activity_level"], variance=0.15, minimum=0.0, maximum=1.0
    )
    steps_walked = int(patient.get("steps_walked", 0))
    activity_state = patient.get("activity_state") or _classify_activity(activity_level)

    if random() < 0.05:
        heart_rate += 12 * random()
    if random() < 0.04:
        spo2 -= 4 * random()
    if random() < 0.02:
        temperature += 0.6 * random()
    if random() < 0.03:
        activity_level = min(1.0, activity_level + 0.35 * random())

    target_state = _classify_activity(activity_level)
    if target_state != activity_state and random() < 0.4:
        activity_state = target_state
    elif random() < 0.06 and activity_state == "sleeping":
        # Occasional stirring
        activity_state = "sitting"

    step_gain = int(max(0, 4 + activity_level * 25 + random() * 10))
    steps_walked += step_gain
    patient["steps_walked"] = steps_walked
    patient["activity_level"] = activity_level
    patient["activity_state"] = activity_state

    fall_detected = random() < 0.005

    return Vitals(
        heart_rate=heart_rate,
        spo2=spo2,
        temperature=temperature,
        ambient_temperature=ambient_temperature,
        humidity=humidity,
        activity_level=activity_level,
        activity_state=activity_state,
        steps_walked=steps_walked,
        fall_detected=fall_detected,
    )


def _bounded_jitter(base: float, variance: float, minimum: float, maximum: float) -> float:
    delta = (random() - 0.5) * 2 * variance
    value = base + delta
    return max(minimum, min(maximum, value))


def _classify_activity(activity_level: float) -> str:
    if activity_level < 0.12:
        return "sleeping"
    if activity_level < 0.28:
        return "sitting"
    if activity_level < 0.68:
        return "walking"
    return "exercising"


def _describe_vitals(vitals: Vitals) -> str:
    return (
        f"HR {vitals.heart_rate:.0f} bpm | SpO₂ {vitals.spo2:.0f}% | "
        f"Temp {vitals.temperature:.1f}°C | Room {vitals.ambient_temperature:.1f}°C | "
        f"Humidity {vitals.humidity:.0f}% | Activity {vitals.activity_level:.2f} ({vitals.activity_state}) | "
        f"Steps {vitals.steps_walked}"
    )


async def _agentic_patient_scan(question: str, device_id: str | None) -> str:
    """Call Ollama to reason over all patients with an agentic chain style prompt."""

    vitals_list = data_store.get_latest_vitals()
    if not vitals_list:
        return "No vitals available yet for Ollama."

    patient_lines: list[str] = []
    for vitals in vitals_list:
        flags = _vital_flags(vitals.vitals)
        age = (datetime.utcnow() - vitals.timestamp).total_seconds()
        freshness = "live" if age < 15 else f"{int(age)}s old"
        status = f"Alerts: {', '.join(flags)}" if flags else "Within thresholds"
        patient_lines.append(
            (
                f"- Device {vitals.device_id} ({vitals.patient_hash[:8]}): "
                f"{_describe_vitals(vitals.vitals)} [{freshness}] | {status}"
            )
        )

    system_prompt = (
        "You are an on-call clinician assistant. First scan all patients, flag risks, "
        "then answer the doctor's question clearly with device ids and brief rationale."
    )
    focus_note = (
        f"Prioritize the status for device {device_id} before broader notes."
        if device_id
        else "Highlight any patients needing attention before summarizing overall status."
    )
    user_prompt = (
        f"Doctor question: {question}\n\nPatient snapshot:\n"
        + "\n".join(patient_lines)
        + "\n\nRules:\n- Use only the provided snapshot.\n"
        "- Surface alerts, falls, or threshold breaches first.\n"
        "- Keep answers concise and actionable.\n"
        + focus_note
    )

    try:
        async with httpx.AsyncClient(timeout=20) as client:
            response = await client.post(
                OLLAMA_ENDPOINT,
                json={
                    "model": OLLAMA_MODEL,
                    "stream": False,
                    "messages": [
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_prompt},
                    ],
                },
            )
            response.raise_for_status()
            payload = response.json()
            message = payload.get("message") if isinstance(payload, dict) else None
            if isinstance(message, dict) and message.get("content"):
                return str(message["content"]).strip()
            content = payload.get("response") if isinstance(payload, dict) else None
            return str(content).strip() if content else "Ollama returned no content."
    except httpx.HTTPError as exc:
        logger.warning("Ollama request failed: %s", exc)
        return "Ollama is unavailable right now; please try again later or use the quick assistant."


def _vital_flags(vitals: Vitals) -> list[str]:
    flags: list[str] = []
    if vitals.heart_rate > thresholds.max_heart_rate:
        flags.append("High heart rate")
    if vitals.spo2 < thresholds.min_spo2:
        flags.append("Low SpO₂")
    if vitals.temperature > thresholds.max_temperature:
        flags.append("Fever")
    if vitals.fall_detected:
        flags.append("Fall detected")
    return flags


def _build_patient_response(question: str, vitals: VitalsMessage) -> str:
    flags = _vital_flags(vitals.vitals)
    summary = _describe_vitals(vitals.vitals)
    age = (datetime.utcnow() - vitals.timestamp).total_seconds()
    age_note = "Live" if age < 15 else f"{int(age)}s old"
    details = f"Patient {vitals.device_id} ({vitals.patient_hash[:8]}): {summary} ({age_note})."

    if flags:
        status = " Alerts: " + ", ".join(flags) + "."
    else:
        status = " All readings are inside configured thresholds."

    question_lower = question.lower()
    if "oxygen" in question_lower or "spo2" in question_lower:
        focus = f" Current SpO₂ is {vitals.vitals.spo2:.0f}% with threshold {thresholds.min_spo2}% minimum."
    elif "heart" in question_lower:
        focus = (
            f" Heart rate is {vitals.vitals.heart_rate:.0f} bpm; "
            f"alerts trigger above {thresholds.max_heart_rate} bpm."
        )
    elif "temperature" in question_lower or "fever" in question_lower:
        focus = (
            f" Temperature is {vitals.vitals.temperature:.1f}°C; "
            f"threshold is {thresholds.max_temperature}°C."
        )
    elif "humidity" in question_lower or "environment" in question_lower:
        focus = (
            f" Room temp is {vitals.vitals.ambient_temperature:.1f}°C with "
            f"{vitals.vitals.humidity:.0f}% humidity nearby."
        )
    elif "activity" in question_lower or "movement" in question_lower:
        focus = (
            " Activity monitor shows "
            f"a score of {vitals.vitals.activity_level:.2f} ({vitals.vitals.activity_state}) with "
            + ("a fall alert." if vitals.vitals.fall_detected else "no falls detected.")
        )
    elif "step" in question_lower or "walk" in question_lower:
        focus = (
            f" Recorded steps today: {vitals.vitals.steps_walked:,} while {vitals.vitals.activity_state}. "
            "Counts climb faster when movement spikes."
        )
    elif "fall" in question_lower:
        focus = (
            " Fall detection is "
            + ("active." if vitals.vitals.fall_detected else "clear.")
        )
    else:
        focus = ""

    return details + status + focus

