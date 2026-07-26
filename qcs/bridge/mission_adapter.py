#!/usr/bin/env python3

import json
import math
import os
import time
from pathlib import Path
import sys

import rclpy
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import (
    CommandBool,
    SetMode,
    WaypointClear,
    WaypointPush,
    WaypointSetCurrent,
)
from rclpy.node import Node

from qgc_plan import load_qgc_plan, validate_hover_plan


SERVICE_TIMEOUT = 20.0
CONNECTION_TIMEOUT = 20.0
AUTO_MISSION_MODE = "AUTO.MISSION"
EARTH_RADIUS_METERS = 6378137.0
TRAY_LANDING_TOLERANCE_METERS = 0.75


def horizontal_distance_meters(first, second):
    latitude_scale = math.pi * EARTH_RADIUS_METERS / 180.0
    mean_latitude = math.radians((first[0] + second[0]) / 2.0)
    north = (first[0] - second[0]) * latitude_scale
    east = (
        (first[1] - second[1])
        * latitude_scale
        * math.cos(mean_latitude)
    )
    return math.hypot(north, east)


def expected_tray_target():
    encoded = os.environ.get("ARECADA_TRAY_LANDING_TARGET")
    if not encoded:
        return None

    try:
        target = json.loads(encoded)
        latitude = float(target["latitude"])
        longitude = float(target["longitude"])
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise ValueError("트레이 착륙 목표 좌표 형식이 잘못되었습니다") from error

    return latitude, longitude


def validate_tray_landing(items, target):
    if items[-1]["command"] != 21:
        raise ValueError("트레이 미션의 마지막 항목은 LAND여야 합니다")

    landing = (items[-1]["latitude"], items[-1]["longitude"])
    approach = (items[-2]["latitude"], items[-2]["longitude"])
    landing_error = horizontal_distance_meters(landing, target)
    approach_error = horizontal_distance_meters(approach, target)

    if landing_error > TRAY_LANDING_TOLERANCE_METERS:
        raise ValueError(
            f"LAND 지점이 트레이에서 {landing_error:.2f}m 벗어났습니다"
        )
    if approach_error > TRAY_LANDING_TOLERANCE_METERS:
        raise ValueError(
            f"최종 접근 지점이 트레이에서 {approach_error:.2f}m 벗어났습니다"
        )
    if items[-2]["altitude"] <= 0:
        raise ValueError("트레이 최종 접근 고도는 0m보다 높아야 합니다")


def emit(status, message, **payload):
    print(
        json.dumps(
            {"status": status, "message": message, **payload},
            separators=(",", ":"),
        ),
        flush=True,
    )


def inspect_plan(plan_path):
    items, summary = load_qgc_plan(plan_path)
    tray_target = expected_tray_target()
    if items[-1]["command"] == 21:
        if len(items) < 3:
            raise ValueError("착륙 미션은 이륙·접근·착륙 항목이 필요합니다")
        if tray_target is not None:
            validate_tray_landing(items, tray_target)
    else:
        validate_hover_plan(items)
    resolved_path, item_count, start, end = summary
    waypoints = [
        {
            "sequence": index,
            "command": item["command"],
            "latitude": item["latitude"],
            "longitude": item["longitude"],
            "altitude": item["altitude"],
        }
        for index, item in enumerate(items, start=1)
    ]
    emit(
        "ready",
        (
            f"트레이 착륙 경로 {item_count}개 항목 검증 완료"
            if tray_target is not None
            else f"{item_count}개 웨이포인트 검증 완료"
        ),
        path=str(resolved_path),
        count=item_count,
        start=start,
        end=end,
        waypoints=waypoints,
    )
    return items


def to_ros_waypoint(item):
    waypoint = Waypoint()
    waypoint.frame = item["frame"]
    waypoint.command = item["command"]
    waypoint.is_current = item["is_current"]
    waypoint.autocontinue = item["autocontinue"]
    waypoint.param1 = item["param1"]
    waypoint.param2 = item["param2"]
    waypoint.param3 = item["param3"]
    waypoint.param4 = item["param4"]
    waypoint.x_lat = item["latitude"]
    waypoint.y_long = item["longitude"]
    waypoint.z_alt = item["altitude"]
    return waypoint


class MissionClient(Node):
    def __init__(self):
        super().__init__("arecada_mission_adapter")
        self.state = None
        self.create_subscription(
            MavrosState,
            "/mavros/state",
            self.on_state,
            10,
        )
        self.clear_client = self.create_client(
            WaypointClear,
            "/mavros/mission/clear",
        )
        self.push_client = self.create_client(
            WaypointPush,
            "/mavros/mission/push",
        )
        self.current_client = self.create_client(
            WaypointSetCurrent,
            "/mavros/mission/set_current",
        )
        self.arm_client = self.create_client(
            CommandBool,
            "/mavros/cmd/arming",
        )
        self.mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode",
        )

    def on_state(self, message):
        self.state = message

    def wait_for_connection(self):
        emit("connecting", "PX4 MAVROS 연결 확인 중")
        deadline = time.monotonic() + CONNECTION_TIMEOUT

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state is not None and self.state.connected:
                emit("connected", "PX4 MAVROS 연결 완료")
                return

        raise TimeoutError("PX4 MAVROS 연결 시간이 초과됐습니다")

    def call(self, client, request, description):
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            raise TimeoutError(f"MAVROS {description} 서비스를 찾을 수 없습니다")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=SERVICE_TIMEOUT,
        )

        if not future.done():
            raise TimeoutError(f"MAVROS {description} 응답 시간이 초과됐습니다")
        if future.exception() is not None:
            raise RuntimeError(f"MAVROS {description} 실패: {future.exception()}")

        return future.result()

    def upload(self, items):
        emit("uploading", "기존 PX4 미션 삭제 중")
        clear_response = self.call(
            self.clear_client,
            WaypointClear.Request(),
            "미션 삭제",
        )
        if not clear_response.success:
            raise RuntimeError("기존 미션 삭제에 실패했습니다")

        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = [to_ros_waypoint(item) for item in items]
        emit("uploading", f"{len(items)}개 웨이포인트 업로드 중")
        push_response = self.call(
            self.push_client,
            request,
            "미션 업로드",
        )

        if (
            not push_response.success
            or push_response.wp_transfered != len(items)
        ):
            raise RuntimeError(
                f"{len(items)}개 중 {push_response.wp_transfered}개만 전송됐습니다"
            )

        current_request = WaypointSetCurrent.Request()
        current_request.wp_seq = 0
        current_response = self.call(
            self.current_client,
            current_request,
            "첫 웨이포인트 선택",
        )
        if not current_response.success:
            raise RuntimeError("첫 웨이포인트 선택에 실패했습니다")

        emit(
            "uploaded",
            f"{len(items)}개 웨이포인트 PX4 업로드 완료",
            count=len(items),
        )

    def start(self, items):
        emit("starting", "선택한 미션을 PX4에 다시 확인 중")
        self.upload(items)
        arm_request = CommandBool.Request()
        arm_request.value = True
        emit("starting", "기체 ARM 요청 중")
        arm_response = self.call(
            self.arm_client,
            arm_request,
            "ARM",
        )
        if not arm_response.success:
            raise RuntimeError("PX4가 ARM 요청을 거부했습니다")

        mode_request = SetMode.Request()
        mode_request.custom_mode = AUTO_MISSION_MODE
        emit("starting", "AUTO.MISSION 모드 전환 중")
        mode_response = self.call(
            self.mode_client,
            mode_request,
            "AUTO.MISSION 전환",
        )
        if not mode_response.mode_sent:
            raise RuntimeError("PX4가 AUTO.MISSION 모드를 거부했습니다")

        emit("started", "AUTO.MISSION 시작 완료")


def run_action(action, plan_path):
    items = inspect_plan(plan_path)
    if action == "inspect":
        return

    rclpy.init(args=None)
    node = MissionClient()
    try:
        node.wait_for_connection()
        if action == "upload":
            node.upload(items)
        elif action == "start":
            node.start(items)
        else:
            raise ValueError(f"지원하지 않는 작업입니다: {action}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    if len(sys.argv) != 3:
        print(
            "usage: mission_adapter.py <inspect|upload|start> <plan>",
            file=sys.stderr,
        )
        raise SystemExit(2)

    action = sys.argv[1]
    plan_path = Path(sys.argv[2])

    try:
        run_action(action, plan_path)
    except (TimeoutError, RuntimeError, ValueError) as error:
        emit("error", str(error))
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
