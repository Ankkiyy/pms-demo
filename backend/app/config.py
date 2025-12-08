"""Configuration management for the patient monitoring backend.

Environment variables allow swapping brokers and credentials without code changes.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional


@dataclass
class MQTTConfig:
    host: str = os.environ.get("MQTT_HOST", "localhost")
    port: int = int(os.environ.get("MQTT_PORT", 8883))
    username: Optional[str] = os.environ.get("MQTT_USER")
    password: Optional[str] = os.environ.get("MQTT_PASS")
    ca_cert_path: Optional[str] = os.environ.get("MQTT_CA_CERT")
    client_id: str = os.environ.get("MQTT_CLIENT_ID", "backend_service")
    pub_cmd_topic_template: str = os.environ.get(
        "MQTT_CMD_TOPIC_TEMPLATE", "patients/{device_id}/cmd"
    )
    sub_vitals_topic: str = os.environ.get("MQTT_VITALS_TOPIC", "patients/+/vitals")


@dataclass
class DiscoveryConfig:
    """Settings used for UDP-based backend discovery on the local network."""

    listen_port: int = int(os.environ.get("DISCOVERY_PORT", 4211))
    request_magic: str = os.environ.get("DISCOVERY_REQUEST", "PMS_DISCOVER")
    response_magic: str = os.environ.get("DISCOVERY_RESPONSE", "PMS_BACKEND")
    advertised_port: int = int(os.environ.get("DISCOVERY_API_PORT", 8000))
    advertised_scheme: str = os.environ.get("DISCOVERY_SCHEME", "http")


@dataclass
class Thresholds:
    max_heart_rate: float = float(os.environ.get("THRESHOLD_MAX_HR", 120))
    min_spo2: float = float(os.environ.get("THRESHOLD_MIN_SPO2", 92))
    max_temperature: float = float(os.environ.get("THRESHOLD_MAX_TEMP", 38.0))


mqtt_config = MQTTConfig()
discovery_config = DiscoveryConfig()
thresholds = Thresholds()

