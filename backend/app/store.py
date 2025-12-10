"""Thread-safe in-memory storage for vitals and alerts."""
from __future__ import annotations

import asyncio
import threading
from collections import deque
from datetime import datetime
from typing import Deque, Dict, List, Tuple

from asyncio import AbstractEventLoop

from .schemas import Alert, TelemetryMessage, VitalsMessage


class DataStore:
    def __init__(self, max_alerts: int = 1000):
        self._lock = threading.Lock()
        self._latest_vitals: Dict[str, Tuple[datetime, VitalsMessage]] = {}
        self._alerts: Deque[Alert] = deque(maxlen=max_alerts)
        self._telemetry: Dict[str, TelemetryMessage] = {}
        self._listeners: list[asyncio.Queue[VitalsMessage]] = []
        self._event_loop: AbstractEventLoop | None = None

    def update_vitals(self, message: VitalsMessage) -> None:
        with self._lock:
            self._latest_vitals[message.device_id] = (datetime.utcnow(), message)
        self._notify_listeners(message)

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

    def register_listener(self, queue: asyncio.Queue[VitalsMessage]) -> None:
        with self._lock:
            self._listeners.append(queue)
            if not self._event_loop:
                try:
                    self._event_loop = asyncio.get_running_loop()
                except RuntimeError:
                    # No running loop when called from a thread. Caller should set the loop
                    # explicitly via ``set_event_loop``.
                    pass

    def unregister_listener(self, queue: asyncio.Queue[VitalsMessage]) -> None:
        with self._lock:
            if queue in self._listeners:
                self._listeners.remove(queue)

    def set_event_loop(self, loop: AbstractEventLoop) -> None:
        """Store the main event loop for thread-safe notifications."""

        with self._lock:
            self._event_loop = loop

    def _notify_listeners(self, message: VitalsMessage) -> None:
        loop = self._event_loop
        if not loop:
            return

        with self._lock:
            listeners_snapshot = list(self._listeners)

        for queue in listeners_snapshot:
            try:
                asyncio.run_coroutine_threadsafe(queue.put(message), loop)
            except RuntimeError:
                # Event loop might be closed during shutdown.
                continue


data_store = DataStore()

