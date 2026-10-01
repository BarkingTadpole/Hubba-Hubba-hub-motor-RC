#!/usr/bin/env python3
"""Development-only mock of the ESP32 bridge protocol."""

from __future__ import annotations

import argparse
import json
import select
import socket
import time


MOCK_CSV = (
    "schema,boot_id,sample_seq,uptime_ms,vehicle_speed_kmh,gps_speed_kmh,"
    "rpm_fl,rpm_fr,rpm_rl,rpm_rr,imu_yaw_dps,tv_target_yaw_dps,tv_yaw_error_dps\n"
    "1,1234,1,1000,24.2,24.4,1258,1245,1252,1247,0.42,0.0,-0.42\n"
    "1,1234,2,1100,24.5,24.6,1264,1251,1258,1253,0.38,0.0,-0.38\n"
).encode()


def telemetry(elapsed: float) -> dict:
    rpm = 1250 + int(80 * (elapsed % 2.0))
    return {
        "protocol": 1, "uptime_ms": int(elapsed * 1000), "state": "DISARMED",
        "armed": False, "inhibited": False, "inhibit_flags": 0,
        "inhibit_count": 0, "initial_arm_cycle_ready": True, "drive_mode": "AWD",
        "calibration": {"active": False, "sampling": False, "kind": "none",
                        "step": 0, "total_steps": 0, "values_us": [0, 0, 0],
                        "prompt": "", "message": "Mock calibration idle"},
        "rc": {
            "throttle": {"valid": True, "us": 1514, "age_ms": 4},
            "steering": {"valid": True, "us": 1518, "age_ms": 5},
            "arm": {"valid": True, "us": 2049, "age_ms": 7},
            "tv": {"valid": True, "us": 2047, "age_ms": 8},
        },
        "steering": {
            "servo_us": 1518, "filter_us": 1518, "filter_active": True,
            "rejected_spikes": 2, "speed_valid": True, "speed_kmh": 24.3,
            "limited": False, "requested_deg": 0.0, "maximum_deg": 5.45,
            "servo_command_deg": 0.0, "left_wheel_deg": -1.64,
            "right_wheel_deg": 1.64, "average_wheel_deg": 0.0,
        },
        "rpm": {"ppr": 7, "wheels": [
            {"valid": True, "rpm": rpm + 8, "hz": 146.8, "edges": 21, "window_us": 143000},
            {"valid": True, "rpm": rpm - 5, "hz": 145.3, "edges": 20, "window_us": 138000},
            {"valid": True, "rpm": rpm + 2, "hz": 146.0, "edges": 21, "window_us": 144000},
            {"valid": True, "rpm": rpm - 3, "hz": 145.6, "edges": 20, "window_us": 137000},
        ]},
        "imu": {"valid": True, "bias_calibrated": True, "yaw_dps": 0.42,
                "accel_mps2": [0.02, -0.03, 9.80], "gyro_dps": [0.1, -0.1, 0.42]},
        "tv": {"configured": False, "requested": "OFF", "active": False,
               "active_mode": "OFF", "reason": "disabled in configuration",
               "target_yaw_dps": 0.0, "yaw_error_dps": -0.42,
               "side_rpm_error": 0.0, "predicted_lateral_mps2": 0.0,
               "lateral_demand": 0.0, "front_relief": 0.0,
               "correction": [0.0, 0.0, 0.0, 0.0]},
        "outputs": [{"throttle_us": 1100, "reverse_us": 1100} for _ in range(4)],
        "dragy": {"initialized": True, "nmea_recent": True, "fix_valid": True,
                  "age_ms": 18, "baud": 9600, "uart_bytes": 48291,
                  "valid_sentences": 1842, "checksum_errors": 1,
                  "parse_errors": 0, "position_valid": True,
                  "latitude_deg": 43.6532, "longitude_deg": -79.3832,
                  "speed_valid": True, "speed_kmh": 24.6,
                  "course_valid": True, "course_deg": 87.4,
                  "altitude_valid": True, "altitude_m": 76.2,
                  "satellites": 18, "hdop": 0.7,
                  "utc_time_ms": 45319000, "date_ddmmyy": 250826,
                  "i2c_scan_complete": True,
                  "i2c_addresses": [0x6A, 0x42, 0x0D],
                  "compass_candidate_present": True,
                  "compass_data_supported": False},
        "logging": {"initialized": True, "mounted": True, "manual_control": True, "recording": False,
                    "full": False, "faulted": False, "last_errno": 0,
                    "rate_hz": 5, "interval_ms": 200, "boot_id": 1234,
                    "samples": 2, "write_errors": 0, "bytes": len(MOCK_CSV),
                    "capacity_bytes": 2800000, "free_bytes": 2700000},
        "config": {"loaded_from_nvs": True, "reverse_limit_percent": 10,
                   "drive_smoothing_percent": 100,
                   "failsafe_enabled": False,
                   "failsafe_us": 1565, "failsafe_window_us": 10,
                   "motor_poles": 14, "rpm_ppr": 7, "steering_trim_deg": 0.0,
                   "steering_smoothing_ms": 60, "steering_speed_limit": True,
                   "steering_lateral_g": 1.0, "tv_authority_percent": 10,
                   "tv_front_relief_percent": 20, "tv_turn_yaw_gain_dps": 180.0,
                   "tv_turn_rpm_gain": 0.2, "tv_yaw_kp": 0.00025,
                   "tv_yaw_ki": 0.00004, "tv_rpm_kp": 0.2, "imu_yaw_sign": 1,
                   "permanent_arm_latch": True,
                   "logging_rate_hz": 5},
    }


def run(port: int) -> None:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", port))
    listener.listen(1)
    print(f"Mock ESP32 listening on 127.0.0.1:{port}", flush=True)
    recording = False
    while True:
        client, _ = listener.accept()
        with client:
            client.sendall(b"HELLO rc_car protocol=1 access=monitor,config,calibration\n")
            started = time.monotonic()
            buffered = b""
            while True:
                readable, _, _ = select.select([client], [], [], 0.2)
                if readable:
                    payload = client.recv(4096)
                    if not payload:
                        break
                    buffered += payload
                    while b"\n" in buffered:
                        line, buffered = buffered.split(b"\n", 1)
                        text = line.decode("ascii")
                        parts = text.split(" ", 3)
                        if len(parts) == 4 and parts[0] == "C":
                            client.sendall(f"R {parts[1]} OK mock configuration saved\n".encode())
                        elif text.startswith("L ") and " READ " in text:
                            log_parts = text.split()
                            offset = int(log_parts[3])
                            length = min(8192, int(log_parts[4]))
                            chunk = MOCK_CSV[offset:offset + length]
                            client.sendall(f"B {log_parts[1]} {len(MOCK_CSV)} {offset} {len(chunk)}\n".encode() + chunk)
                        elif text.startswith("L ") and text.endswith(" CLEAR"):
                            recording = False
                            log_parts = text.split()
                            client.sendall(f"R {log_parts[1]} OK mock CSV log cleared\n".encode())
                        elif text.startswith("L ") and text.split()[-1] in {"START", "STOP"}:
                            recording = text.split()[-1] == "START"
                            client.sendall(f"R {text.split()[1]} OK mock recording {'started' if recording else 'stopped'}\n".encode())
                snapshot = telemetry(time.monotonic() - started)
                snapshot["logging"]["recording"] = recording
                message = "T " + json.dumps(snapshot, separators=(",", ":")) + "\n"
                try:
                    client.sendall(message.encode())
                except OSError:
                    break


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=3333)
    run(parser.parse_args().port)


if __name__ == "__main__":
    main()
