"""Pydantic schemas shared between the API and MQTT handler."""
from __future__ import annotations

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


class Vitals(BaseModel):
    heart_rate: float = Field(..., description="Beats per minute")
    spo2: float = Field(..., description="Blood oxygen saturation percentage")
    temperature: float = Field(..., description="Body temperature in Celsius")
    fall_detected: bool = Field(..., description="True if fall detection triggered")


class VitalsMessage(BaseModel):
    device_id: str = Field(..., description="Unique patient node identifier")
    patient_hash: str = Field(..., description="Hashed patient identifier")
    timestamp: datetime = Field(..., description="ISO timestamp from the device")
    vitals: Vitals


class Alert(BaseModel):
    device_id: str
    reason: str
    vitals: Vitals
    timestamp: datetime


class Command(BaseModel):
    cmd: str = Field(..., description="Actuator command name")
    reason: Optional[str] = Field(None, description="Why the command was issued")
    timestamp: datetime = Field(default_factory=lambda: datetime.utcnow())
