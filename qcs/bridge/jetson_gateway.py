#!/usr/bin/env python3

import copy
import json
import math
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32

from px4_bridge import Px4ElectronBridge

try:
    from mission_msgs.msg import MissionState
except ImportError:
    MissionState = None


PROJECT_ROOT = Path(__file__).resolve().parent.parent
MISSION_ADAPTER = PROJECT_ROOT / "bridge" / "mission_adapter.py"
MISSION_DIRECTORY = PROJECT_ROOT / ".generated" / "jetson_gateway"
MISSION_PATH = MISSION_DIRECTORY / "remote.plan"
HOST = os.environ.get("JETSON_GCS_HOST", "0.0.0.0")
PORT = int(os.environ.get("JETSON_GCS_PORT", "8765"))


class JetsonBridge(Px4ElectronBridge):
    def __init__(self):
        self.snapshot_lock = threading.Lock()
        self.snapshot = {
            "type": "telemetry",
            "connected": False,
        }
        super().__init__()
        self.state["missionControl"] = {
            "state": "",
            "intentProfile": "",
            "targetOffset": [None, None, None],
            "yawErrorDegrees": None,
        }
        self.create_subscription(
            PointStamped,
            "/target/center_point",
            self.on_target_offset,
            10,
        )
        self.create_subscription(
            Float32,
            "/landing/yaw_error_deg",
            self.on_yaw_error,
            10,
        )
        if MissionState is not None:
            self.create_subscription(
                MissionState,
                "/mission/state",
                self.on_mission_state,
                10,
            )

    def on_target_offset(self, message):
        self.state["missionControl"]["targetOffset"] = [
            float(value) if math.isfinite(value) else None
            for value in [
                message.point.x,
                message.point.y,
                message.point.z,
            ]
        ]

    def on_yaw_error(self, message):
        value = float(message.data)
        self.state["missionControl"]["yawErrorDegrees"] = (
            value if math.isfinite(value) else None
        )

    def on_mission_state(self, message):
        self.state["missionControl"]["state"] = message.state
        self.state["missionControl"]["intentProfile"] = message.intent_profile

    def publish_state(self):
        is_connected = time.monotonic() - self.last_px4_message < 2.0
        self.state["setpoint"]["valid"] = (
            self.state["setpoint"]["valid"]
            and time.monotonic() - self.last_position_setpoint_message < 1.0
        )
        snapshot = {
            "type": "telemetry",
            "connected": is_connected,
            **copy.deepcopy(self.state),
        }
        with self.snapshot_lock:
            self.snapshot = snapshot

    def telemetry(self):
        with self.snapshot_lock:
            return copy.deepcopy(self.snapshot)


class GatewayState:
    def __init__(self, node):
        self.node = node
        self.mission_lock = threading.Lock()

    def run_mission(self, action, plan):
        if action not in {"inspect", "upload", "start"}:
            return {
                "ok": False,
                "error": f"지원하지 않는 미션 작업입니다: {action}",
            }
        if not isinstance(plan, dict):
            return {
                "ok": False,
                "error": "QGroundControl Plan JSON이 필요합니다",
            }
        if not self.mission_lock.acquire(blocking=False):
            return {
                "ok": False,
                "error": "다른 미션 작업이 진행 중입니다",
            }

        try:
            MISSION_DIRECTORY.mkdir(parents=True, exist_ok=True)
            MISSION_PATH.write_text(
                f"{json.dumps(plan, ensure_ascii=False, indent=2)}\n",
                encoding="utf-8",
            )
            adapter_environment = {
                **os.environ,
                **self.tray_target_environment(),
            }
            completed = subprocess.run(
                [
                    sys.executable,
                    str(MISSION_ADAPTER),
                    action,
                    str(MISSION_PATH),
                ],
                cwd=PROJECT_ROOT,
                capture_output=True,
                text=True,
                env=adapter_environment,
                timeout=120,
                check=False,
            )
            result = None
            for line in completed.stdout.splitlines():
                try:
                    result = json.loads(line)
                except json.JSONDecodeError:
                    continue

            error = None
            if completed.returncode != 0:
                if isinstance(result, dict):
                    error = result.get("message")
                if not error:
                    error = completed.stderr.strip() or "미션 작업 실패"

            return {
                "ok": completed.returncode == 0,
                "result": result,
                "error": error,
            }
        except subprocess.TimeoutExpired:
            return {
                "ok": False,
                "error": "미션 작업 응답 시간이 초과됐습니다",
            }
        finally:
            self.mission_lock.release()

    def tray_target_environment(self):
        telemetry = self.node.telemetry()
        reference = telemetry.get("estimate", {}).get("referencePosition")
        tray = telemetry.get("simulation", {}).get("trayPosition")
        if not isinstance(reference, list) or not isinstance(tray, list):
            return {}
        if len(reference) < 2 or len(tray) < 2:
            return {}
        values = reference[:2] + tray[:2]
        if not all(
            isinstance(value, (int, float)) and math.isfinite(value)
            for value in values
        ):
            return {}

        earth_radius = 6378137.0
        latitude = reference[0] + math.degrees(tray[0] / earth_radius)
        longitude = reference[1] + math.degrees(
            tray[1]
            / (
                earth_radius
                * math.cos(math.radians(reference[0]))
            )
        )
        return {
            "ARECADA_TRAY_LANDING_TARGET": json.dumps(
                {
                    "latitude": latitude,
                    "longitude": longitude,
                },
                separators=(",", ":"),
            ),
        }


class GatewayHandler(BaseHTTPRequestHandler):
    server_version = "ARECADA-Jetson-Gateway/1.0"

    def log_message(self, message, *args):
        sys.stderr.write(
            f"[Jetson gateway] {self.address_string()} "
            f"{message % args}\n"
        )

    def send_json(self, status, payload):
        encoded = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(encoded)

    def read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 2 * 1024 * 1024:
            raise ValueError("요청 본문 크기가 잘못되었습니다")
        try:
            return json.loads(self.rfile.read(length))
        except json.JSONDecodeError as error:
            raise ValueError("올바른 JSON 요청이 아닙니다") from error

    def do_GET(self):
        if self.path == "/health":
            telemetry = self.server.gateway.node.telemetry()
            last_message = self.server.gateway.node.last_px4_message
            last_message_age = (
                round(time.monotonic() - last_message, 3)
                if last_message > 0.0
                else None
            )
            self.send_json(
                200,
                {
                    "ok": True,
                    "px4Connected": telemetry.get("connected", False),
                    "lastPx4MessageAgeSeconds": last_message_age,
                    "dds": {
                        "transport": os.environ.get(
                            "PX4_DDS_TRANSPORT",
                            "unknown",
                        ),
                        "port": int(os.environ.get("PX4_DDS_PORT", "8888")),
                        "device": os.environ.get(
                            "PX4_DDS_DEVICE",
                            "/dev/ttyTHS1",
                        ),
                        "baudrate": int(
                            os.environ.get("PX4_DDS_BAUDRATE", "921600")
                        ),
                    },
                },
            )
            return
        if self.path == "/status":
            self.send_json(
                200,
                {
                    **self.server.gateway.node.telemetry(),
                    "gatewayConnected": True,
                },
            )
            return
        self.send_json(404, {"ok": False, "error": "Not found"})

    def do_POST(self):
        try:
            payload = self.read_json()
        except ValueError as error:
            self.send_json(400, {"ok": False, "error": str(error)})
            return

        if self.path == "/command":
            command = payload.get("command")
            if not isinstance(command, dict):
                self.send_json(
                    400,
                    {"ok": False, "error": "command 객체가 필요합니다"},
                )
                return
            self.server.gateway.node.commands.put(command)
            self.send_json(202, {"ok": True, "accepted": True})
            return

        prefix = "/mission/"
        if self.path.startswith(prefix):
            action = self.path[len(prefix):]
            result = self.server.gateway.run_mission(
                action,
                payload.get("plan"),
            )
            self.send_json(200 if result["ok"] else 409, result)
            return

        self.send_json(404, {"ok": False, "error": "Not found"})


class GatewayServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, gateway):
        super().__init__(address, GatewayHandler)
        self.gateway = gateway


def main():
    rclpy.init(args=None)
    node = JetsonBridge()
    gateway = GatewayState(node)
    server = GatewayServer((HOST, PORT), gateway)
    server_thread = threading.Thread(
        target=server.serve_forever,
        daemon=True,
    )
    server_thread.start()
    print(
        f"Jetson GCS gateway listening on http://{HOST}:{PORT}",
        file=sys.stderr,
        flush=True,
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
