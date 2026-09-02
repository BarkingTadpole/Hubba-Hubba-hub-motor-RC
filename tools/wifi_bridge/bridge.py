#!/usr/bin/env python3
"""Browser bridge for the ESP32 RC-car telemetry/configuration protocol."""

from __future__ import annotations

import argparse
import json
import select
import socket
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


DISCOVERY_REQUEST = b"RC_CAR_DISCOVER_V1"
DISCOVERY_RESPONSE = "RC_CAR_V1"


class EspLink:
    def __init__(self, host: str | None, control_port: int, discovery_port: int):
        self._fixed_host = host
        self._control_port = control_port
        self._discovery_port = discovery_port
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._connected_host: str | None = None
        self._last_known_host: str | None = host
        self._telemetry: dict[str, Any] | None = None
        self._telemetry_received_at = 0.0
        self._last_error = "not connected"
        self._hello = ""
        self._rx_bytes = 0
        self._telemetry_count = 0
        self._json_error_count = 0
        self._last_protocol_line = ""
        self._next_request_id = 1
        self._pending: dict[int, tuple[threading.Event, dict[str, Any]]] = {}
        self._pending_logs: dict[int, tuple[threading.Event, dict[str, Any]]] = {}

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="esp-link", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            active = self._socket
        if active is not None:
            try:
                active.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            active.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            age_ms = None
            if self._telemetry_received_at:
                age_ms = round((time.time() - self._telemetry_received_at) * 1000)
            transport_connected = self._socket is not None
            telemetry_received = self._telemetry is not None
            telemetry_fresh = (transport_connected and telemetry_received and
                               age_ms is not None and age_ms < 3000)
            return {
                # Keep connected for the dashboard, but make it mean usable
                # live telemetry instead of merely an open TCP socket.
                "connected": telemetry_fresh,
                "transport_connected": transport_connected,
                "telemetry_received": telemetry_received,
                "telemetry_fresh": telemetry_fresh,
                "esp_host": self._connected_host,
                "last_known_esp_host": self._last_known_host,
                "last_error": self._last_error,
                "telemetry_age_ms": age_ms,
                "hello": self._hello,
                "rx_bytes": self._rx_bytes,
                "telemetry_count": self._telemetry_count,
                "json_error_count": self._json_error_count,
                "last_protocol_line": self._last_protocol_line,
                "telemetry": self._telemetry,
            }

    def send_command(self, command: str, timeout: float = 4.0) -> dict[str, Any]:
        if not self._remote_command_allowed(command):
            return {
                "ok": False,
                "message": "Only guarded configuration, calibration, and maintenance commands are allowed",
            }

        with self._lock:
            active = self._socket
            if active is None:
                return {"ok": False, "message": "ESP32 is not connected"}
            request_id = self._next_request_id
            self._next_request_id += 1
            completed = threading.Event()
            result: dict[str, Any] = {}
            self._pending[request_id] = (completed, result)

        try:
            with self._send_lock:
                active.sendall(f"C {request_id} {command}\n".encode("ascii"))
        except OSError as exc:
            with self._lock:
                self._pending.pop(request_id, None)
            return {"ok": False, "message": f"send failed: {exc}"}

        if not completed.wait(timeout):
            with self._lock:
                self._pending.pop(request_id, None)
            return {"ok": False, "message": "ESP32 command timed out"}
        return result

    def read_log_chunk(self, offset: int, length: int,
                       timeout: float = 8.0) -> dict[str, Any]:
        if offset < 0 or length < 0 or length > 8192:
            return {"ok": False, "message": "invalid log byte range"}
        with self._lock:
            active = self._socket
            if active is None:
                return {"ok": False, "message": "ESP32 is not connected"}
            request_id = self._next_request_id
            self._next_request_id += 1
            completed = threading.Event()
            result: dict[str, Any] = {}
            self._pending_logs[request_id] = (completed, result)
        try:
            with self._send_lock:
                active.sendall(f"L {request_id} READ {offset} {length}\n".encode("ascii"))
        except OSError as exc:
            with self._lock:
                self._pending_logs.pop(request_id, None)
            return {"ok": False, "message": f"log request failed: {exc}"}
        if not completed.wait(timeout):
            with self._lock:
                self._pending_logs.pop(request_id, None)
            return {"ok": False, "message": "ESP32 log read timed out"}
        return result

    def clear_log(self, timeout: float = 5.0) -> dict[str, Any]:
        with self._lock:
            active = self._socket
            if active is None:
                return {"ok": False, "message": "ESP32 is not connected"}
            request_id = self._next_request_id
            self._next_request_id += 1
            completed = threading.Event()
            result: dict[str, Any] = {}
            self._pending[request_id] = (completed, result)
        try:
            with self._send_lock:
                active.sendall(f"L {request_id} CLEAR\n".encode("ascii"))
        except OSError as exc:
            with self._lock:
                self._pending.pop(request_id, None)
            return {"ok": False, "message": f"clear request failed: {exc}"}
        if not completed.wait(timeout):
            with self._lock:
                self._pending.pop(request_id, None)
            return {"ok": False, "message": "ESP32 log clear timed out"}
        return result

    @staticmethod
    def _remote_command_allowed(command: str) -> bool:
        calibration_commands = {
            "cal receiver", "cal steering", "cal arm", "cal tv", "cal imu",
            "cal capture", "cal esc arm", "cal esc max", "cal esc min",
            "cal manual", "cal cancel",
        }
        return command.startswith("config ") or command in calibration_commands or command in {
            "tv enable", "tv disable", "disarm config"
        }

    def _discover(self) -> tuple[str, int] | None:
        discovery = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            discovery.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            discovery.bind(("", 0))
            discovery.settimeout(0.8)
            discovery.sendto(DISCOVERY_REQUEST, ("255.255.255.255", self._discovery_port))
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                try:
                    payload, address = discovery.recvfrom(128)
                except socket.timeout:
                    return None
                text = payload.decode("ascii", errors="replace")
                parts = text.split()
                if len(parts) == 2 and parts[0] == DISCOVERY_RESPONSE:
                    return address[0], int(parts[1])
        except (OSError, ValueError):
            return None
        finally:
            discovery.close()
        return None

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                remembered_host = self._last_known_host

            targets: list[tuple[str, int]] = []
            if self._fixed_host is not None:
                targets.append((self._fixed_host, self._control_port))
            elif remembered_host is not None:
                # Windows Mobile Hotspot can stop forwarding UDP broadcast
                # discovery after a station reconnects even though direct
                # unicast traffic to the same DHCP address works. Try the last
                # confirmed address first and rediscover only if it fails.
                targets.append((remembered_host, self._control_port))
            else:
                discovered = self._discover()
                if discovered is not None:
                    targets.append(discovered)

            active: socket.socket | None = None
            target: tuple[str, int] | None = None
            errors: list[str] = []
            for candidate in targets:
                try:
                    active = socket.create_connection(candidate, timeout=3.0)
                    active.setblocking(False)
                    target = candidate
                    break
                except OSError as exc:
                    errors.append(
                        f"connect to {candidate[0]}:{candidate[1]} failed: {exc}"
                    )

            if active is None and self._fixed_host is None and remembered_host is not None:
                discovered = self._discover()
                if discovered is None:
                    errors.append("UDP rediscovery received no response")
                elif discovered not in targets:
                    try:
                        active = socket.create_connection(discovered, timeout=3.0)
                        active.setblocking(False)
                        target = discovered
                    except OSError as exc:
                        errors.append(
                            f"connect to {discovered[0]}:{discovered[1]} failed: {exc}"
                        )

            if active is None or target is None:
                with self._lock:
                    self._last_error = ("; ".join(errors) if errors else
                                        "ESP32 not discovered; retrying")
                self._stop.wait(1.0)
                continue

            with self._lock:
                self._socket = active
                self._connected_host = target[0]
                self._last_known_host = target[0]
                self._telemetry = None
                self._telemetry_received_at = 0.0
                self._hello = ""
                self._rx_bytes = 0
                self._telemetry_count = 0
                self._json_error_count = 0
                self._last_protocol_line = ""
                self._last_error = "TCP connected; waiting for telemetry"
            print(f"esp: TCP connected to {target[0]}:{target[1]}", flush=True)
            disconnect_reason = "connection ended"
            try:
                disconnect_reason = self._read_session(active)
            finally:
                stopping = self._stop.is_set()
                try:
                    active.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                active.close()
                with self._lock:
                    if self._socket is active:
                        self._socket = None
                    self._connected_host = None
                    self._last_error = ("bridge stopped" if stopping else
                                        f"{disconnect_reason}; retrying")
                    pending = list(self._pending.values())
                    self._pending.clear()
                    pending_logs = list(self._pending_logs.values())
                    self._pending_logs.clear()
                for completed, result in pending:
                    result.update(ok=False, message="ESP32 connection lost")
                    completed.set()
                for completed, result in pending_logs:
                    result.update(ok=False, message="ESP32 connection lost")
                    completed.set()
                if not stopping:
                    print(f"esp: {disconnect_reason}; retrying", flush=True)
            self._stop.wait(0.5)

    def _read_session(self, active: socket.socket) -> str:
        buffered = b""
        binary: dict[str, Any] | None = None
        last_rx = time.monotonic()
        while not self._stop.is_set():
            try:
                readable, _, exceptional = select.select([active], [], [active], 1.0)
            except (OSError, ValueError) as exc:
                return f"socket wait failed: {exc}"
            if exceptional:
                return "ESP32 socket exception"
            if not readable:
                if time.monotonic() - last_rx > 5.0:
                    return "ESP32 TCP socket was silent for 5 seconds"
                continue
            try:
                payload = active.recv(8192)
            except BlockingIOError:
                continue
            except OSError as exc:
                return f"receive failed: {exc}"
            if not payload:
                return "ESP32 closed the TCP connection"
            last_rx = time.monotonic()
            with self._lock:
                self._rx_bytes += len(payload)
            buffered += payload
            while buffered:
                if binary is not None:
                    needed = binary["count"] - len(binary["data"])
                    take = min(needed, len(buffered))
                    binary["data"].extend(buffered[:take])
                    buffered = buffered[take:]
                    if len(binary["data"]) == binary["count"]:
                        self._complete_binary(binary)
                        binary = None
                    else:
                        break
                    continue
                if b"\n" not in buffered:
                    break
                raw_line, buffered = buffered.split(b"\n", 1)
                line = raw_line.decode("utf-8", errors="replace").rstrip("\r")
                binary = self._process_line(line)
            if len(buffered) > 65536:
                return "protocol receive buffer exceeded 65536 bytes"
        return "bridge stopping"

    def _complete_binary(self, binary: dict[str, Any]) -> None:
        with self._lock:
            pending = self._pending_logs.pop(binary["request_id"], None)
        if pending is None:
            return
        completed, result = pending
        result.update(ok=True, total=binary["total"], offset=binary["offset"],
                      data=bytes(binary["data"]))
        completed.set()

    def _process_line(self, line: str) -> dict[str, Any] | None:
        with self._lock:
            self._last_protocol_line = line[:240]
        if line.startswith("HELLO "):
            with self._lock:
                self._hello = line
                self._last_error = "protocol handshake received; waiting for telemetry"
            return None

        if line.startswith("T "):
            try:
                telemetry = json.loads(line[2:])
            except json.JSONDecodeError as exc:
                with self._lock:
                    self._last_error = f"invalid telemetry JSON: {exc}"
                    self._json_error_count += 1
                return None
            with self._lock:
                self._telemetry = telemetry
                self._telemetry_received_at = time.time()
                self._telemetry_count += 1
                first = self._telemetry_count == 1
                self._last_error = ""
            if first:
                print("esp: first valid telemetry frame received", flush=True)
            return None

        if line.startswith("B "):
            parts = line.split()
            if len(parts) != 5:
                return None
            try:
                request_id, total, offset, count = map(int, parts[1:])
            except ValueError:
                return None
            if count < 0 or count > 8192:
                return None
            binary = {"request_id": request_id, "total": total,
                      "offset": offset, "count": count, "data": bytearray()}
            if count == 0:
                self._complete_binary(binary)
                return None
            return binary

        if line.startswith("R "):
            parts = line.split(" ", 3)
            if len(parts) < 4:
                return
            try:
                request_id = int(parts[1])
            except ValueError:
                return
            with self._lock:
                pending = self._pending.pop(request_id, None)
                log_pending = (None if pending is not None else
                               self._pending_logs.pop(request_id, None))
            if pending is not None:
                completed, result = pending
                result.update(ok=parts[2] == "OK", message=parts[3])
                completed.set()
            elif log_pending is not None:
                completed, result = log_pending
                result.update(ok=False, message=parts[3])
                completed.set()
        return None


class DashboardHandler(BaseHTTPRequestHandler):
    link: EspLink
    static_directory: Path

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/api/state":
            self._json_response(HTTPStatus.OK, self.link.snapshot())
            return
        if self.path == "/api/config":
            state = self.link.snapshot()
            telemetry = state.get("telemetry") or {}
            config = telemetry.get("config")
            if not state.get("telemetry_fresh") or not isinstance(config, dict):
                self._json_response(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    {"ok": False, "message": "Fresh ESP32 configuration is unavailable"},
                )
                return
            self._json_response(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "source": ("nvs" if config.get("loaded_from_nvs") is True else
                               "compiled_defaults" if config.get("loaded_from_nvs") is False else
                               "device_runtime"),
                    "config": config,
                    "drive_mode": telemetry.get("drive_mode"),
                    "tv_configured": (telemetry.get("tv") or {}).get("configured"),
                    "logging": telemetry.get("logging") or {},
                },
            )
            return
        if self.path in {"/", "/index.html"}:
            self._file_response("index.html", "text/html; charset=utf-8")
            return
        if self.path == "/app.js":
            self._file_response("app.js", "text/javascript; charset=utf-8")
            return
        if self.path == "/style.css":
            self._file_response("style.css", "text/css; charset=utf-8")
            return
        if self.path == "/api/log.csv":
            self._log_download()
            return
        if self.path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        if self.path == "/api/disarm":
            result = self.link.send_command("disarm config")
            status = HTTPStatus.OK if result["ok"] else HTTPStatus.CONFLICT
            self._json_response(status, result)
            return
        if self.path == "/api/log/clear":
            result = self.link.clear_log()
            status = HTTPStatus.OK if result["ok"] else HTTPStatus.CONFLICT
            self._json_response(status, result)
            return
        if self.path not in {"/api/config", "/api/calibration"}:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_error(HTTPStatus.BAD_REQUEST)
            return
        if length <= 0 or length > 4096:
            self.send_error(HTTPStatus.BAD_REQUEST)
            return
        try:
            request = json.loads(self.rfile.read(length))
            command = request["command"]
            if not isinstance(command, str):
                raise TypeError
        except (json.JSONDecodeError, KeyError, TypeError):
            self._json_response(HTTPStatus.BAD_REQUEST,
                                {"ok": False, "message": "Expected JSON command string"})
            return
        result = self.link.send_command(command.strip().lower())
        status = HTTPStatus.OK if result["ok"] else HTTPStatus.CONFLICT
        self._json_response(status, result)

    def log_message(self, format_string: str, *args: Any) -> None:
        print(f"browser: {self.address_string()} - {format_string % args}")

    def _file_response(self, filename: str, content_type: str) -> None:
        try:
            payload = (self.static_directory / filename).read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _log_download(self) -> None:
        state = self.link.snapshot()
        telemetry = state.get("telemetry") or {}
        logging = telemetry.get("logging") or {}
        if logging and int(logging.get("samples") or 0) <= 0:
            storage_exhausted = (bool(logging.get("full")) and
                                 int(logging.get("free_bytes") or 0) <= 16384)
            message = ("The device log has no telemetry samples. The dedicated "
                       "log storage reports no usable free space; while DISARMED, "
                       "use Recover log storage to rebuild it."
                       if storage_exhausted else
                       "The device log has not recorded a telemetry sample yet.")
            self._json_response(
                HTTPStatus.CONFLICT,
                {"ok": False, "message": message, "logging": logging},
            )
            return
        first = self.link.read_log_chunk(0, 0)
        if not first.get("ok"):
            self._json_response(HTTPStatus.SERVICE_UNAVAILABLE, first)
            return
        total = int(first["total"])
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/csv; charset=utf-8")
        self.send_header("Content-Disposition", "attachment; filename=rc_car_telemetry.csv")
        self.send_header("Content-Length", str(total))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        offset = 0
        while offset < total:
            result = self.link.read_log_chunk(offset, min(8192, total - offset))
            if not result.get("ok") or not result.get("data"):
                return
            data = result["data"]
            self.wfile.write(data)
            offset += len(data)

    def _json_response(self, status: HTTPStatus, value: dict[str, Any]) -> None:
        payload = json.dumps(value, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--esp", default="auto",
                        help="ESP32 IPv4 address, or 'auto' for UDP discovery")
    parser.add_argument("--control-port", type=int, default=3333)
    parser.add_argument("--discovery-port", type=int, default=3334)
    parser.add_argument("--listen", default="127.0.0.1",
                        help="browser HTTP bind address; use 0.0.0.0 for LAN access")
    parser.add_argument("--http-port", type=int, default=8080)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    link = EspLink(None if args.esp == "auto" else args.esp,
                   args.control_port, args.discovery_port)
    link.start()

    DashboardHandler.link = link
    DashboardHandler.static_directory = Path(__file__).with_name("web")
    server = ThreadingHTTPServer((args.listen, args.http_port), DashboardHandler)
    print(f"RC car dashboard: http://{args.listen}:{args.http_port}")
    print("Press Ctrl+C to stop. Wi-Fi control never exposes arm or drive commands.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        link.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
