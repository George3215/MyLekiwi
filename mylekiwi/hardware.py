from __future__ import annotations

import math
from pathlib import Path

import draccus
import numpy as np
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

ARM = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
]
ALL = ARM + ["arm_gripper"]


def load_calibration(path: Path) -> dict[str, MotorCalibration]:
    with path.open(encoding="utf-8") as file, draccus.config_type("json"):
        calibration = draccus.load(dict[str, MotorCalibration], file)
    missing = [name for name in ALL if name not in calibration]
    if missing:
        raise ValueError(f"motor calibration is missing {missing}")
    return calibration


def make_bus(port: str, calibration: dict[str, MotorCalibration]) -> FeetechMotorsBus:
    motors = {
        name: Motor(i + 1, "sts3215", MotorNormMode.DEGREES if name in ARM else MotorNormMode.RANGE_0_100)
        for i, name in enumerate(ALL)
    }
    return FeetechMotorsBus(
        port=port,
        protocol_version=1,
        motors=motors,
        calibration={name: calibration[name] for name in ALL},
    )


def read_arm(bus: FeetechMotorsBus) -> np.ndarray:
    values = bus.sync_read("Present_Position", ARM, num_retry=2)
    return np.asarray([float(values[name]) for name in ARM])


def hold_current(bus: FeetechMotorsBus) -> None:
    values = bus.sync_read("Present_Position", ALL, num_retry=2)
    for name in ALL:
        bus.write("Goal_Position", name, float(values[name]), num_retry=1)


def raw_from_degrees(value: float, calibration: MotorCalibration) -> int:
    midpoint = (float(calibration.range_min) + float(calibration.range_max)) / 2
    return int(value * 4095 / 360 + midpoint)


def safe_raw_bounds(calibration: MotorCalibration, margin_fraction: float = 0.05) -> tuple[int, int]:
    low, high = int(calibration.range_min), int(calibration.range_max)
    margin = math.ceil((high - low) * margin_fraction)
    return low + margin, high - margin
