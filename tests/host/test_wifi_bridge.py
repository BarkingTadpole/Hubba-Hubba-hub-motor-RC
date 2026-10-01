import ast
import importlib.util
import json
import re
import socket
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BRIDGE_PATH = ROOT / "tools" / "wifi_bridge" / "bridge.py"
WIFI_CONTROL_PATH = ROOT / "main" / "wifi_control.c"
KCONFIG_PATH = ROOT / "main" / "Kconfig.projbuild"
MAIN_CMAKE_PATH = ROOT / "main" / "CMakeLists.txt"
DEFAULT_CONFIG_PATH = ROOT / "main" / "default_config.h"
CONFIG_STORE_PATH = ROOT / "main" / "config_store.c"
WEB_APP_PATH = ROOT / "tools" / "wifi_bridge" / "web" / "app.js"
WEB_INDEX_PATH = ROOT / "tools" / "wifi_bridge" / "web" / "index.html"
SPEC = importlib.util.spec_from_file_location("wifi_bridge", BRIDGE_PATH)
bridge = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(bridge)

TEST_CSV = (b"schema,boot_id,sample_seq,uptime_ms,vehicle_speed_kmh\n" +
            b"1,7,1,100,1.5\n" * 700)


class FirmwareTelemetryCapacityTest(unittest.TestCase):
    def test_json_schema_and_value_headroom_fit_transmit_buffer(self):
        source = WIFI_CONTROL_PATH.read_text(encoding="utf-8")
        size_match = re.search(r"#define TELEMETRY_BUFFER_SIZE\s+(\d+)", source)
        self.assertIsNotNone(size_match)
        buffer_size = int(size_match.group(1))

        function_start = source.index("static int format_telemetry(")
        start = source.index("int written = snprintf(", function_start)
        end = source.index("(long long)(snapshot.captured_at_us / 1000)", start)
        format_block = source[start:end]
        literals = re.findall(r'"(?:\\.|[^"\\])*"', format_block)
        telemetry_format = "".join(ast.literal_eval(value) for value in literals)

        # Values replace the printf conversion tokens, so retain generous room
        # beyond the fixed schema to prevent another silent truncation failure.
        self.assertGreaterEqual(buffer_size, len(telemetry_format) + 2048)


class FirmwareSoftApViewerTest(unittest.TestCase):
    def test_softap_and_direct_http_viewer_are_the_default_path(self):
        source = WIFI_CONTROL_PATH.read_text(encoding="utf-8")
        kconfig = KCONFIG_PATH.read_text(encoding="utf-8")
        cmake = MAIN_CMAKE_PATH.read_text(encoding="utf-8")
        app = WEB_APP_PATH.read_text(encoding="utf-8")
        index = WEB_INDEX_PATH.read_text(encoding="utf-8")

        self.assertRegex(
            kconfig,
            r"config RC_WIFI_CONTROL_ENABLED\s+bool [^\n]+\s+default y",
        )
        self.assertIn('default "RC-Car-ESP32"', kconfig)
        self.assertIn('default "rc-car-viewer"', kconfig)
        self.assertIn("esp_netif_create_default_wifi_ap", source)
        self.assertIn("WIFI_MODE_AP", source)
        self.assertNotIn("WIFI_MODE_STA", source)
        self.assertIn("httpd_start", source)
        for route in (
            "/api/state", "/api/config", "/api/calibration", "/api/disarm",
            "/api/log.csv", "/api/log/clear", "/api/log/start", "/api/log/stop",
        ):
            self.assertIn(route, source)
        self.assertIn("esp_http_server", cmake)
        self.assertIn('RENAME_TO "web_index"', cmake)
        self.assertIn('RENAME_TO "web_app"', cmake)
        self.assertIn('RENAME_TO "web_style"', cmake)
        self.assertIn("payload.protocol === 1", app)
        self.assertIn("direct_http: true", app)
        self.assertIn("config drive smoothing", app)
        self.assertIn('data-builder="drive-smoothing"', index)
        self.assertIn("Browser → ESP32", index)
        self.assertNotIn("Browser → bridge", index)

    def test_requested_first_boot_profile_is_compiled(self):
        defaults = DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
        store = CONFIG_STORE_PATH.read_text(encoding="utf-8")
        expected_macros = {
            "DEFAULT_RC_THROTTLE_FAILSAFE_ENABLED": "1U",
            "DEFAULT_REVERSE_LIMIT_PERCENT": "100U",
            "DEFAULT_DRIVE_SMOOTHING_PERCENT": "100U",
            "DEFAULT_STEERING_SMOOTHING_MS": "10U",
            "DEFAULT_STEERING_SPEED_LIMIT_ENABLED": "1U",
            "DEFAULT_TV_ENABLED": "1U",
            "DEFAULT_TV_AUTHORITY_PERCENT": "25U",
            "DEFAULT_TV_FRONT_RELIEF_PERCENT": "20U",
            "DEFAULT_IMU_YAW_SIGN": "(-1)",
            "DEFAULT_TELEMETRY_LOG_RATE_HZ": "1U",
            "DEFAULT_PERMANENT_ARM_LATCH_ENABLED": "1",
        }
        for name, value in expected_macros.items():
            self.assertRegex(defaults, rf"#define\s+{name}\s+{re.escape(value)}")
            self.assertIn(name, store)
        self.assertIn(".drivetrain_mode = DRIVETRAIN_AWD", store)
        self.assertIn(".motor_poles = 14", store)
        self.assertIn(".rpm_pulses_per_revolution = 7", store)
        self.assertIn(".steering_trim_tenths_deg = 0", store)
        self.assertIn(".tv_turn_yaw_gain_dps = 180.0f", store)
        self.assertIn(".tv_turn_rpm_gain = 0.20f", store)
        self.assertIn(".tv_yaw_kp = 0.00025f", store)
        self.assertIn(".tv_yaw_ki = 0.00004f", store)
        self.assertIn(".tv_rpm_kp = 0.20f", store)


class EmptyLogLink:
    def snapshot(self):
        return {
            "telemetry": {
                "logging": {
                    "samples": 0,
                    "bytes": 1956,
                    "capacity_bytes": 2756608,
                    "free_bytes": 0,
                    "full": True,
                }
            }
        }


class EmptyLogHttpTest(unittest.TestCase):
    def test_header_only_full_volume_reports_recovery_action(self):
        class Handler(bridge.DashboardHandler):
            pass

        Handler.link = EmptyLogLink()
        Handler.static_directory = ROOT / "tools" / "wifi_bridge" / "web"
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            url = f"http://127.0.0.1:{server.server_address[1]}/api/log.csv"
            with self.assertRaises(urllib.error.HTTPError) as context:
                urllib.request.urlopen(url)
            with context.exception as response:
                self.assertEqual(response.code, 409)
                result = json.load(response)
            self.assertFalse(result["ok"])
            self.assertIn("Recover log storage", result["message"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


class FakeEsp:
    def __init__(self):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.stop.set()
        self.listener.close()
        self.thread.join(timeout=2)

    def run(self):
        try:
            client, _ = self.listener.accept()
        except OSError:
            return
        with client:
            client.settimeout(0.1)
            client.sendall(b'HELLO rc_car protocol=1 access=monitor,config,calibration\n')
            client.sendall(
                b'T {"protocol":1,"state":"DISARMED","drive_mode":"AWD",'
                b'"config":{"loaded_from_nvs":true,"permanent_arm_latch":true,'
                b'"logging_rate_hz":5},"tv":{"configured":false},'
                b'"calibration":{"active":false,"sampling":false,'
                b'"kind":"none","step":0,"total_steps":0,'
                b'"values_us":[0,0,0],"prompt":"","message":"ready"},'
                b'"logging":{"rate_hz":5,"samples":700,"bytes":24500,'
                b'"capacity_bytes":2756608,"free_bytes":2700000},'
                b'"dragy":{"initialized":true,"nmea_recent":true,'
                b'"fix_valid":true,"speed_kph":42.5}}\n'
            )
            buffered = b""
            while not self.stop.is_set():
                try:
                    payload = client.recv(4096)
                except socket.timeout:
                    continue
                if not payload:
                    return
                buffered += payload
                while b"\n" in buffered:
                    line, buffered = buffered.split(b"\n", 1)
                    text = line.decode()
                    parts = text.split(" ", 3)
                    if len(parts) == 4 and parts[0] == "C":
                        client.sendall(f"R {parts[1]} OK test saved\n".encode())
                    elif text.startswith("L ") and " READ " in text:
                        log_parts = text.split()
                        offset = int(log_parts[3])
                        length = int(log_parts[4])
                        chunk = TEST_CSV[offset:offset + length]
                        client.sendall(
                            f"B {log_parts[1]} {len(TEST_CSV)} {offset} {len(chunk)}\n".encode()
                            + chunk
                        )
                    elif text.startswith("L ") and text.endswith(" CLEAR"):
                        client.sendall(f"R {text.split()[1]} OK test log cleared\n".encode())
                    elif text.startswith("L ") and text.split()[-1] in {"START", "STOP"}:
                        client.sendall(f"R {text.split()[1]} OK test log {text.split()[-1].lower()}\n".encode())


class ReconnectingFakeEsp:
    def __init__(self):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.stop = threading.Event()
        self.second_connection = threading.Event()
        self.connection_count = 0
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.stop.set()
        self.listener.close()
        self.thread.join(timeout=2)

    def run(self):
        while not self.stop.is_set() and self.connection_count < 2:
            try:
                client, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.connection_count += 1
            session = self.connection_count
            with client:
                client.sendall(b"HELLO rc_car protocol=1 access=monitor,config,calibration\n")
                client.sendall(
                    f'T {{"protocol":1,"session":{session},"state":"DISARMED"}}\n'.encode()
                )
                if session == 1:
                    continue
                self.second_connection.set()
                while not self.stop.wait(0.1):
                    try:
                        client.sendall(
                            f'T {{"protocol":1,"session":{session},'
                            '"state":"DISARMED"}\n'.encode()
                        )
                    except OSError:
                        return


class OneShotDiscoveryLink(bridge.EspLink):
    def __init__(self, control_port):
        super().__init__(None, control_port, 3334)
        self.discovery_calls = 0

    def _discover(self):
        self.discovery_calls += 1
        if self.discovery_calls == 1:
            return "127.0.0.1", self._control_port
        return None


class ReconnectTest(unittest.TestCase):
    def test_last_known_address_recovers_when_broadcast_discovery_stops(self):
        fake = ReconnectingFakeEsp()
        link = OneShotDiscoveryLink(fake.port)
        fake.start()
        link.start()
        try:
            self.assertTrue(fake.second_connection.wait(4.0))
            deadline = time.time() + 2.0
            while time.time() < deadline:
                state = link.snapshot()
                telemetry = state.get("telemetry") or {}
                if state["connected"] and telemetry.get("session") == 2:
                    break
                time.sleep(0.02)
            state = link.snapshot()
            self.assertTrue(state["connected"])
            self.assertEqual(state["telemetry"]["session"], 2)
            self.assertEqual(state["last_known_esp_host"], "127.0.0.1")
            self.assertEqual(link.discovery_calls, 1)
        finally:
            link.stop()
            fake.close()


class BridgeIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeEsp()
        self.fake.start()
        self.link = bridge.EspLink("127.0.0.1", self.fake.port, 3334)
        self.link.start()
        deadline = time.time() + 3
        while time.time() < deadline:
            if self.link.snapshot()["telemetry"] is not None:
                break
            time.sleep(0.02)
        self.assertTrue(self.link.snapshot()["connected"])

    def tearDown(self):
        self.link.stop()
        self.fake.close()

    def test_tcp_telemetry_and_guarded_command(self):
        state = self.link.snapshot()
        self.assertEqual(state["telemetry"]["state"], "DISARMED")
        self.assertTrue(state["transport_connected"])
        self.assertTrue(state["telemetry_fresh"])
        self.assertTrue(state["telemetry"]["dragy"]["fix_valid"])
        self.assertEqual(state["telemetry"]["dragy"]["speed_kph"], 42.5)
        self.assertTrue(self.link.send_command("config reverse 12")["ok"])
        self.assertTrue(self.link.send_command("config logging rate 5")["ok"])
        self.assertTrue(self.link.send_command("config arm-latch off")["ok"])
        self.assertTrue(self.link.send_command("cal steering")["ok"])
        self.assertTrue(self.link.send_command("disarm config")["ok"])
        self.assertFalse(self.link.send_command("arm")["ok"])
        self.assertFalse(self.link.send_command("disarm")["ok"])
        self.assertFalse(self.link.send_command("cal unknown")["ok"])
        chunk = self.link.read_log_chunk(0, 8192)
        self.assertTrue(chunk["ok"])
        self.assertEqual(chunk["total"], len(TEST_CSV))
        self.assertEqual(chunk["data"], TEST_CSV[:8192])
        self.assertTrue(self.link.clear_log()["ok"])

    def test_http_state_static_page_and_config_proxy(self):
        class Handler(bridge.DashboardHandler):
            pass

        Handler.link = self.link
        Handler.static_directory = ROOT / "tools" / "wifi_bridge" / "web"
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_address[1]}"
        try:
            with urllib.request.urlopen(base + "/api/state") as response:
                payload = json.load(response)
            self.assertTrue(payload["connected"])
            with urllib.request.urlopen(base + "/") as response:
                index_html = response.read()
            self.assertIn(b"RC Car Monitor", index_html)
            self.assertIn(b"Calibration console", index_html)
            self.assertIn(b"cal esc arm", index_html)
            with urllib.request.urlopen(base + "/app.js") as response:
                app_js = response.read()
            self.assertIn(b"dragy", app_js)
            self.assertIn(b"config logging rate", app_js)
            self.assertIn(b"config arm-latch", app_js)
            self.assertIn(b"cycle-drivetrain", app_js)
            self.assertIn(b"Recover log storage", app_js)
            self.assertIn(b"disarm-for-config", app_js)
            self.assertIn(b"/api/calibration", app_js)
            self.assertIn(b"calibrationConfirmations", app_js)
            with urllib.request.urlopen(base + "/api/config") as response:
                saved_config = json.load(response)
            self.assertTrue(saved_config["ok"])
            self.assertEqual(saved_config["source"], "nvs")
            self.assertTrue(saved_config["config"]["permanent_arm_latch"])
            with urllib.request.urlopen(base + "/api/log.csv") as response:
                self.assertEqual(response.read(), TEST_CSV)
            request = urllib.request.Request(
                base + "/api/config",
                data=json.dumps({"command": "config rpm ppr 7"}).encode(),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(request) as response:
                result = json.load(response)
            self.assertTrue(result["ok"])

            calibration_request = urllib.request.Request(
                base + "/api/calibration",
                data=json.dumps({"command": "cal receiver"}).encode(),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(calibration_request) as response:
                self.assertTrue(json.load(response)["ok"])

            disarm_request = urllib.request.Request(
                base + "/api/disarm", data=b"", method="POST"
            )
            with urllib.request.urlopen(disarm_request) as response:
                self.assertTrue(json.load(response)["ok"])

            rejected = urllib.request.Request(
                base + "/api/config",
                data=json.dumps({"command": "arm"}).encode(),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with self.assertRaises(urllib.error.HTTPError) as context:
                urllib.request.urlopen(rejected)
            with context.exception as response:
                self.assertEqual(response.code, 409)
                self.assertFalse(json.load(response)["ok"])

            clear_request = urllib.request.Request(
                base + "/api/log/clear", data=b"", method="POST"
            )
            with urllib.request.urlopen(clear_request) as response:
                self.assertTrue(json.load(response)["ok"])
            for action in ("start", "stop"):
                request = urllib.request.Request(base + "/api/log/" + action, data=b"", method="POST")
                with urllib.request.urlopen(request) as response:
                    result = json.load(response)
                    self.assertTrue(result["ok"])
                    self.assertEqual(result["message"], "test log " + action)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
