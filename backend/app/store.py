"""Thread-safe in-memory storage for vitals and alerts."""
from __future__ import annotations

import threading
from collections import deque
from datetime import datetime
from typing import Deque, Dict, List, Tuple

from .schemas import Alert, TelemetryMessage, VitalsMessage


class DataStore:
    def __init__(self, max_alerts: int = 1000):
        self._lock = threading.Lock()
        self._latest_vitals: Dict[str, Tuple[datetime, VitalsMessage]] = {}
        self._alerts: Deque[Alert] = deque(maxlen=max_alerts)
        self._telemetry: Dict[str, TelemetryMessage] = {}

    def update_vitals(self, message: VitalsMessage) -> None:
        with self._lock:
            self._latest_vitals[message.device_id] = (datetime.utcnow(), message)

    def record_alert(self, alert: Alert) -> None:
        with self._lock:
            self._alerts.append(alert)

    def get_latest_vitals(self) -> List[VitalsMessage]:
        with self._lock:
            return [msg for _, msg in self._latest_vitals.values()]

    def get_vitals_for_device(self, device_id: str) -> VitalsMessage | None:
        with self._lock:
            pair = self._latest_vitals.get(device_id)
            return pair[1] if pair else None

    def get_alerts(self) -> List[Alert]:
        with self._lock:
            return list(self._alerts)

    def record_telemetry(self, message: TelemetryMessage) -> None:
        with self._lock:
            self._telemetry[message.device_id] = message

    def get_latest_telemetry(self) -> List[TelemetryMessage]:
        with self._lock:
            return list(self._telemetry.values())


data_store = DataStore()

