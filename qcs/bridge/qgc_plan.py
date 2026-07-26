import json
import math
from pathlib import Path


def _finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} 값이 숫자가 아닙니다")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{label} 값이 유효하지 않습니다")
    return value


def load_qgc_plan(plan_path):
    resolved_path = Path(plan_path).expanduser().resolve()
    try:
        plan = json.loads(resolved_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"Plan 파일을 찾을 수 없습니다: {resolved_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError("QGroundControl Plan JSON 형식이 잘못되었습니다") from error

    if plan.get("fileType") != "Plan":
        raise ValueError("QGroundControl Plan 파일이 아닙니다")

    mission = plan.get("mission")
    source_items = mission.get("items") if isinstance(mission, dict) else None
    if not isinstance(source_items, list) or not source_items:
        raise ValueError("Plan에 미션 항목이 없습니다")

    items = []
    for index, source in enumerate(source_items, start=1):
        if not isinstance(source, dict) or source.get("type") != "SimpleItem":
            raise ValueError(f"{index}번 항목은 지원하지 않는 복합 미션입니다")

        params = source.get("params")
        if not isinstance(params, list) or len(params) < 7:
            raise ValueError(f"{index}번 항목의 params 형식이 잘못되었습니다")

        items.append(
            {
                "frame": int(_finite_number(source.get("frame"), "frame")),
                "command": int(
                    _finite_number(source.get("command"), "command")
                ),
                "is_current": index == 1,
                "autocontinue": bool(source.get("autoContinue", True)),
                "param1": _finite_number(params[0] or 0, "param1"),
                "param2": _finite_number(params[1] or 0, "param2"),
                "param3": _finite_number(params[2] or 0, "param3"),
                "param4": (
                    float("nan")
                    if params[3] is None
                    else _finite_number(params[3], "param4")
                ),
                "latitude": _finite_number(params[4], "latitude"),
                "longitude": _finite_number(params[5], "longitude"),
                "altitude": _finite_number(params[6], "altitude"),
            }
        )

    start = [items[0]["latitude"], items[0]["longitude"]]
    end = [items[-1]["latitude"], items[-1]["longitude"]]
    return items, (resolved_path, len(items), start, end)


def validate_hover_plan(items):
    if len(items) < 2:
        raise ValueError("미션에는 최소 2개 항목이 필요합니다")
    if items[-1]["command"] != 16:
        raise ValueError("마지막 항목은 호버 웨이포인트여야 합니다")
    if items[-1]["altitude"] <= 0:
        raise ValueError("마지막 호버 고도는 0m보다 높아야 합니다")
