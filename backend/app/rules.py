"""Alerting rules that evaluate incoming vitals."""
from __future__ import annotations

from typing import Optional

from .config import thresholds
from .schemas import Vitals


def evaluate_alert(vitals: Vitals) -> Optional[str]:
    """Return an alert reason or ``None`` if vitals are within range."""
    if vitals.heart_rate > thresholds.max_heart_rate:
        return "HIGH_HEART_RATE"
    if vitals.spo2 < thresholds.min_spo2:
        return "LOW_SPO2"
    if vitals.temperature > thresholds.max_temperature:
        return "FEVER"
    if vitals.fall_detected:
        return "FALL_DETECTED"
    return None
