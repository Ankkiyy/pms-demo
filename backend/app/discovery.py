"""Simple UDP discovery responder so ESP32 clients can find the backend."""
from __future__ import annotations

import socket
import threading
from typing import Optional

from .config import discovery_config


class DiscoveryResponder:
    """Listens for broadcast discovery packets and replies with API details."""

    def __init__(self) -> None:
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3)

    def _serve(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", discovery_config.listen_port))
            sock.settimeout(1.0)

            while not self._stop_event.is_set():
                try:
                    data, addr = sock.recvfrom(256)
                except socket.timeout:
                    continue
                except OSError:
                    break

                if data.decode(errors="ignore").strip() != discovery_config.request_magic:
                    continue

                payload = self._build_response()
                try:
                    sock.sendto(payload.encode(), addr)
                except OSError:
                    continue

    def _build_response(self) -> str:
        host_ip = self._local_ip()
        return f"{discovery_config.response_magic} {discovery_config.advertised_scheme}://{host_ip}:{discovery_config.advertised_port}"

    @staticmethod
    def _local_ip() -> str:
        """Best-effort discovery of the primary local IP address."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except OSError:
            return "127.0.0.1"


discovery_responder = DiscoveryResponder()
