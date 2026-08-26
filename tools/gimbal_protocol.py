"""Shared serial protocol helpers for the Armtrack gimbal endpoint."""

from __future__ import annotations

import re
import math
from dataclasses import dataclass
from typing import Dict, Optional

YAW_MIN_X10 = 0
YAW_MAX_X10 = 3599
PITCH_MIN_X10 = 2300
PITCH_MAX_X10 = 4050
YAW_SPEED_MIN = -100
YAW_SPEED_MAX = 100
PITCH_SPEED_MIN = -30
PITCH_SPEED_MAX = 30


def parse_angle(value: str, low_x10: int, high_x10: int) -> int:
    text = value.strip().lower()
    if text.startswith("x"):
        raw_text = text[1:]
        if not _DECIMAL_RE.fullmatch(raw_text):
            raise ValueError("angle must be decimal or xNNNN")
        raw = float(raw_text)
        if not math.isfinite(raw) or raw < 0 or raw != math.trunc(raw):
            raise ValueError("raw angle must be an integer")
        result = int(raw)
    else:
        if not _DECIMAL_RE.fullmatch(text):
            raise ValueError("angle must be decimal or xNNNN")
        scaled = float(text) * 10.0
        if not math.isfinite(scaled):
            raise ValueError("angle must be finite")
        result = _round_half_away_from_zero(scaled)
    if not low_x10 <= result <= high_x10:
        raise ValueError(f"angle must be between {low_x10 / 10:g} and {high_x10 / 10:g}")
    return result


_DECIMAL_RE = re.compile(r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)$")


def _round_half_away_from_zero(value: float) -> int:
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def format_angle(angle_x10: int) -> str:
    sign = "-" if angle_x10 < 0 else ""
    value = abs(int(angle_x10))
    return f"{sign}{value // 10}.{value % 10}"


def validate_integer(value: str, low: int, high: int, name: str) -> int:
    try:
        result = int(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer") from exc
    if not low <= result <= high:
        raise ValueError(f"{name} must be between {low} and {high}")
    return result


def command_tick(sequence: int) -> str:
    if sequence < 0:
        raise ValueError("sequence must be non-negative")
    return f"tick {sequence}"


def command_home() -> str:
    return "home"


def command_stop() -> str:
    return "stop"


def command_yaw_speed(rpm: int) -> str:
    return f"yaw {validate_integer(str(rpm), YAW_SPEED_MIN, YAW_SPEED_MAX, 'yaw rpm')}"


def command_yaw_position(angle: str | int) -> str:
    value = angle if isinstance(angle, int) else parse_angle(angle, YAW_MIN_X10, YAW_MAX_X10)
    if not YAW_MIN_X10 <= value <= YAW_MAX_X10:
        raise ValueError("yaw angle must be between 0 and 359.9")
    return f"yawpos {format_angle(value)}"


def command_pitch_position(angle: str | int) -> str:
    value = angle if isinstance(angle, int) else parse_angle(angle, PITCH_MIN_X10, PITCH_MAX_X10)
    if not PITCH_MIN_X10 <= value <= PITCH_MAX_X10:
        raise ValueError("pitch angle must be between 230 and 405")
    return f"pitch {format_angle(value)}"


def command_pitch_speed(rpm: int) -> str:
    return f"pitchspd {validate_integer(str(rpm), PITCH_SPEED_MIN, PITCH_SPEED_MAX, 'pitch rpm')}"


def command_track(yaw_rpm: int, pitch: str | int) -> str:
    yaw = validate_integer(str(yaw_rpm), YAW_SPEED_MIN, YAW_SPEED_MAX, 'yaw rpm')
    value = pitch if isinstance(pitch, int) else parse_angle(pitch, PITCH_MIN_X10, PITCH_MAX_X10)
    if not PITCH_MIN_X10 <= value <= PITCH_MAX_X10:
        raise ValueError("pitch angle must be between 230 and 405")
    return f"track {yaw} {format_angle(value)}"


def command_pose(yaw: str | int, pitch: str | int) -> str:
    yaw_x10 = yaw if isinstance(yaw, int) else parse_angle(yaw, YAW_MIN_X10, YAW_MAX_X10)
    pitch_x10 = pitch if isinstance(pitch, int) else parse_angle(pitch, PITCH_MIN_X10, PITCH_MAX_X10)
    if not YAW_MIN_X10 <= yaw_x10 <= YAW_MAX_X10:
        raise ValueError("yaw angle must be between 0 and 359.9")
    if not PITCH_MIN_X10 <= pitch_x10 <= PITCH_MAX_X10:
        raise ValueError("pitch angle must be between 230 and 405")
    return f"pose {format_angle(yaw_x10)} {format_angle(pitch_x10)}"


@dataclass
class GimbalStatus:
    yaw_current: Optional[str] = None
    yaw_target: Optional[str] = None
    pitch_current: Optional[str] = None
    pitch_target: Optional[str] = None
    homed: Optional[str] = None
    home_fault: Optional[str] = None
    state: Optional[str] = None


_FIELD_RE = re.compile(r"^([^:]+):(.*)$")


def parse_status_line(line: str) -> Dict[str, str]:
    fields: Dict[str, str] = {}
    for part in line.strip().split(","):
        match = _FIELD_RE.match(part.strip())
        if match:
            fields[match.group(1).strip()] = match.group(2).strip()
    return fields


def status_from_line(line: str) -> Optional[GimbalStatus]:
    fields = parse_status_line(line)
    if "yaw_cur" not in fields or "pitch_cur" not in fields:
        return None
    return GimbalStatus(
        yaw_current=fields.get("yaw_cur"),
        yaw_target=fields.get("yaw_tgt"),
        pitch_current=fields.get("pitch_cur"),
        pitch_target=fields.get("pitch_tgt"),
        homed=fields.get("homed"),
        home_fault=fields.get("home_fault"),
        state=fields.get("state"),
    )


def describe_line(line: str) -> str:
    text = line.strip()
    if not text:
        return ""
    if text.startswith("TOCK "):
        parts = text.split()
        if len(parts) == 3:
            return f"tick_reply sequence={parts[1]} uptime_ms={parts[2]}"
    if text.startswith("OK ") or text.startswith("ERR "):
        return f"response={text}"
    fields = parse_status_line(text)
    return "fields " + " ".join(f"{key}={value}" for key, value in fields.items()) if fields else f"text={text}"
