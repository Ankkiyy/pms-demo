"""MQTT client that processes vitals and issues actuator commands."""
from __future__ import annotations

import json
import threading
import time
from datetime import datetime
from typing import Callable, Optional

import paho.mqtt.client as mqtt

from .config import mqtt_config
from .rules import evaluate_alert
from .schemas import Alert, Command, VitalsMessage
from .store import data_store


class MQTTBridge:
    def __init__(self) -> None:
        self._client = mqtt.Client(client_id=mqtt_config.client_id)
        if mqtt_config.username and mqtt_config.password:
            self._client.username_pw_set(mqtt_config.username, mqtt_config.password)
        if mqtt_config.ca_cert_path:
            self._client.tls_set(ca_certs=mqtt_config.ca_cert_path)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    def start(self) -> None:
        """Start the MQTT network loop in a background thread."""
        self._client.connect(mqtt_config.host, mqtt_config.port, keepalive=60)
        self._thread = threading.Thread(target=self._loop_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._client.disconnect()
        if self._thread:
            self._thread.join(timeout=5)

    # MQTT callbacks
    def _on_connect(self, client: mqtt.Client, userdata, flags, rc):
        client.subscribe(mqtt_config.sub_vitals_topic)

    def _on_message(self, client: mqtt.Client, userdata, msg):
        payload = msg.payload.decode()
        try:
            parsed = VitalsMessage.parse_raw(payload)
        except Exception as exc:  # noqa: BLE001 - library raised exceptions vary
            print(f"Failed to parse payload on {msg.topic}: {exc}")
            return

        data_store.update_vitals(parsed)
        reason = evaluate_alert(parsed.vitals)
        if reason:
            alert = Alert(
                device_id=parsed.device_id,
                reason=reason,
                vitals=parsed.vitals,
                timestamp=datetime.utcnow(),
            )
            data_store.record_alert(alert)
            self._publish_command(parsed.device_id, reason)

    def _publish_command(self, device_id: str, reason: str) -> None:
        topic = mqtt_config.pub_cmd_topic_template.format(device_id=device_id)
        cmd = Command(cmd="ALARM_ON", reason=reason)
        self._client.publish(topic, cmd.json())

    def _loop_forever(self):
        while not self._stop_event.is_set():
            self._client.loop(timeout=1.0)
            time.sleep(0.1)


bridge = MQTTBridge()
