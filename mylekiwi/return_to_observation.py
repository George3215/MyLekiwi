"""Move to the configured observation pose. Motor writes require --execute."""

import argparse
import json
import time

import numpy as np

from .config import load_config, project_path
from .hardware import ARM, hold_current, load_calibration, make_bus, raw_from_degrees, read_arm, safe_raw_bounds
from .kinematics import SO101ForwardKinematics


def main() -> None:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--read-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    cfg = load_config()
    robot, execution = cfg["robot"], cfg["execution"]
    calibration = load_calibration(project_path(cfg["paths"]["calibration"]))
    handeye = json.loads(project_path(cfg["paths"]["handeye"]).read_text())
    baseline = json.loads(project_path(cfg["paths"]["floor_baseline"]).read_text())
    offsets = [handeye["joint_offsets_deg"][name] for name in ARM]
    kinematics = SO101ForwardKinematics(project_path(cfg["paths"]["urdf"]), "gripper_link", offsets)
    link_to_tool = np.asarray(baseline["T_gripper_link_gripper_frame"])
    minimum_z = baseline["minimum_tool_z_m"]

    def tool_z(joints) -> float:
        return float((kinematics.forward(joints) @ link_to_tool)[2, 3])

    target = np.asarray(robot["observation_joint_deg"])
    bus = make_bus(robot["port"], calibration)
    try:
        bus.connect(handshake=True)
        start = read_arm(bus)
        path = [start + t * (target - start) for t in np.linspace(0, 1, 201)]
        path_z = [tool_z(joints) for joints in path]
        if min(path_z) < minimum_z:
            raise RuntimeError("current/path is more than 5 mm below the recorded tool-Z baseline")
        print("tool_z", {"current": path_z[0], "path_min": min(path_z), "target": path_z[-1], "minimum": minimum_z})
        print("current", start.tolist(), "target", target.tolist())
        if args.read_only:
            print("READ_ONLY_OK: zero writes")
            return
        hold_current(bus)
        bus.enable_torque(num_retry=1)
        step = execution["repose_step_deg"]
        count = max(1, int(np.ceil(np.max(np.abs(target - start)) / step)))
        for waypoint in (start + t * (target - start) for t in np.linspace(0, 1, count + 1)[1:]):
            for name, value in zip(ARM, waypoint, strict=True):
                low, high = safe_raw_bounds(calibration[name])
                if not low <= raw_from_degrees(value, calibration[name]) <= high:
                    raise RuntimeError(f"{name} outside calibrated safe range")
                if bus.read("Present_Temperature", name, normalize=False, num_retry=1) >= execution["temperature_limit_c"]:
                    raise RuntimeError(f"{name} over temperature")
                if bus.read("Torque_Enable", name, normalize=False, num_retry=1) != 1:
                    raise RuntimeError(f"{name} torque lost")
            bus.sync_write("Goal_Position", dict(zip(ARM, waypoint, strict=True)), num_retry=1)
            time.sleep(execution["repose_step_interval_s"])
            current = read_arm(bus)
            if tool_z(current) < minimum_z:
                raise RuntimeError("measured tool Z crossed the recorded baseline")
            if np.max(np.abs(current - waypoint)) > execution["repose_tracking_tolerance_deg"]:
                raise RuntimeError(f"joint tracking error is too large: target={waypoint.tolist()} actual={current.tolist()}")
        deadline = time.monotonic() + execution["settle_timeout_s"]
        while np.max(np.abs(read_arm(bus) - target)) > execution["settle_tolerance_deg"]:
            if time.monotonic() > deadline:
                raise RuntimeError("observation pose failed to settle")
            time.sleep(0.1)
        bus.write("Goal_Position", "arm_gripper", robot["gripper_open"], num_retry=1)
        print("REPOSE_OK", read_arm(bus).tolist())
    except BaseException:
        if bus.is_connected:
            hold_current(bus)
        raise
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
