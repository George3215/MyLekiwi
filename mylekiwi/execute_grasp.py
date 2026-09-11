"""Execute one frozen grasp plan. Motor writes require explicit --execute."""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from .config import ROOT, load_config, project_path
from .gripper_geometry import GripperFloorGuard
from .hardware import ALL, ARM, hold_current, load_calibration, make_bus, raw_from_degrees, safe_raw_bounds


def log_event(path: Path, stage: str, **values) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps({"time": time.time(), "stage": stage, **values}, default=str) + "\n")


def set_speed(bus, speeds: dict[str, int], log: Path, stage: str) -> None:
    bus.sync_write("Goal_Velocity", speeds, normalize=False, num_retry=1)
    actual = bus.sync_read("Goal_Velocity", ARM, normalize=False, num_retry=2)
    if any(actual[name] != speeds[name] for name in ARM):
        raise RuntimeError(f"arm speed command was not received: {actual}")
    log_event(log, stage, raw=speeds)


def move(bus, target, check_z, log: Path, stage: str, cfg: dict) -> None:
    tolerance = cfg["grasp_tolerance_deg" if stage == "grasp" else "settle_tolerance_deg"]
    start = np.asarray(list(bus.sync_read("Present_Position", ARM, num_retry=2).values()))
    log_event(log, "move_start", move=stage, target=target.tolist(), start=start.tolist())
    for joints in np.linspace(start, target, 101):
        check_z(joints)
    travel = np.abs(target - start)
    speeds = np.maximum(1, np.rint(cfg["arm_speed_raw"] * travel / max(travel.max(), 1e-6))).astype(int)
    set_speed(bus, dict(zip(ARM, map(int, speeds), strict=True)), log, f"{stage}_speed")
    bus.sync_write("Goal_Position", dict(zip(ARM, target, strict=True)), num_retry=1)
    goal = np.asarray(list(bus.sync_read("Goal_Position", ARM, num_retry=2).values()))
    if np.max(np.abs(goal - target)) > 1:
        raise RuntimeError(f"{stage} goal was not received: target={target.tolist()} goal={goal.tolist()}")
    log_event(log, "direct_goal_sent", move=stage, target=target.tolist(), goal=goal.tolist())
    deadline = time.monotonic() + cfg["settle_timeout_s"]
    while True:
        current = np.asarray(list(bus.sync_read("Present_Position", ARM, num_retry=2).values()))
        z = check_z(current)
        error = current - target
        log_event(log, "motion_sample", move=stage, actual=current.tolist(), error_deg=error.tolist(), full_gripper_z_m=z)
        if np.max(np.abs(error)) <= tolerance:
            log_event(log, "stage_settled", move=stage, target=target.tolist(), actual=current.tolist(), tolerance_deg=tolerance)
            return
        if time.monotonic() > deadline:
            raise RuntimeError(f"{stage} failed to settle: target={target.tolist()} actual={current.tolist()}")
        time.sleep(0.1)


def move_gripper(bus, target: float, log: Path, stage: str, tolerance: float) -> None:
    bus.write("Goal_Position", "arm_gripper", target, num_retry=1)
    deadline = time.monotonic() + 5
    while True:
        actual = float(bus.read("Present_Position", "arm_gripper", num_retry=2))
        if abs(actual - target) <= tolerance:
            log_event(log, stage, target=target, actual=actual)
            return
        if time.monotonic() > deadline:
            raise RuntimeError(f"gripper did not reach {target}: actual={actual}")
        time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--read-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--plan", type=Path, default=ROOT / "outputs/plans/latest/plan.json")
    parser.add_argument("--allow-low-score", action="store_true")
    parser.add_argument("--resume-grasp", action="store_true")
    args = parser.parse_args()
    cfg = load_config()
    robot, execution, grasp_cfg = cfg["robot"], cfg["execution"], cfg["grasp"]
    plan = json.loads(args.plan.read_text())
    if plan.get("planner_revision") != 6:
        raise RuntimeError("stale grasp plan; regenerate it with planner revision 6")
    if plan.get("execution_mode") != "offset_descent_then_center_grasp":
        raise RuntimeError(f"plan is not executable: {plan.get('execution_mode')}")
    candidate = plan["grasp_execution"]
    if not candidate.get("ok"):
        raise RuntimeError(f"grasp was rejected: {candidate.get('reason')}")
    if candidate["anygrasp_score"] < 0.2 and not args.allow_low_score:
        raise RuntimeError("AnyGrasp score is too low; explicit --allow-low-score is required")

    observation = np.asarray(plan["observation_joint_deg"])
    targets = {name: np.asarray(value) for name, value in candidate["targets"].items()}
    handeye = json.loads(project_path(cfg["paths"]["handeye"]).read_text())
    offsets = [handeye["joint_offsets_deg"][name] for name in ARM]
    floor_guard = GripperFloorGuard(
        offsets,
        project_path(cfg["paths"]["urdf"]),
        project_path(cfg["paths"]["gripper_hull"]),
        plan["table_z_m"],
        plan["limits"]["floor_margin_m"],
    )
    floor_z = candidate["floor_guard_z_m"] + execution["runtime_floor_reserve_m"]

    def check_z(joints) -> float:
        z = floor_guard.min_z(joints)
        if z < floor_z:
            raise RuntimeError(f"full gripper below floor guard: {z:.4f} < {floor_z:.4f} m")
        return z

    path = [observation, *(targets[name] for name in ("above", "down", "grasp", "lift")), observation]
    for start, end in zip(path, path[1:], strict=False):
        for joints in np.linspace(start, end, 101):
            check_z(joints)
    calibration = load_calibration(project_path(cfg["paths"]["calibration"]))
    for values in targets.values():
        if abs(values[4]) > grasp_cfg["wrist_roll_limit_deg"]:
            raise RuntimeError("joint5 exceeds the configured physical limit")
        for name, value in zip(ARM, values, strict=True):
            low, high = safe_raw_bounds(calibration[name])
            if not low <= raw_from_degrees(value, calibration[name]) <= high:
                raise RuntimeError(f"{name} is outside its calibrated range")

    log = args.plan.parent / f"execution_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.jsonl"
    log_event(log, "start", mode="execute" if args.execute else "read_only", plan=str(args.plan), resume_grasp=args.resume_grasp)
    bus = make_bus(robot["port"], calibration)
    original_speed = None
    try:
        bus.connect(handshake=True)
        state = bus.sync_read("Present_Position", ALL, num_retry=2)
        current = np.asarray([state[name] for name in ARM])
        expected = targets["grasp"] if args.resume_grasp else observation
        tolerance = execution["grasp_tolerance_deg" if args.resume_grasp else "observation_tolerance_deg"]
        if np.max(np.abs(current - expected)) > tolerance:
            message = "current arm is not at grasp" if args.resume_grasp else "arm moved after the single observation"
            raise RuntimeError(message)
        check_z(current)
        log_event(log, "connected", actual_arm=current.tolist(), actual_all=state)
        if args.read_only:
            print(f"{'READ_ONLY_RESUME_OK' if args.resume_grasp else 'READ_ONLY_OK'} log={log}")
            return
        hold_current(bus)
        bus.enable_torque(ALL, num_retry=1)
        original_speed = bus.sync_read("Goal_Velocity", ARM, normalize=False, num_retry=2)
        if not args.resume_grasp:
            move_gripper(bus, robot["gripper_open"], log, "gripper_open_before_motion", execution["gripper_tolerance"])
            for stage in ("above", "down", "grasp"):
                move(bus, targets[stage], check_z, log, stage, execution)
        move_gripper(bus, robot["gripper_close"], log, "gripper_closed_at_grasp", execution["gripper_tolerance"])
        started = time.monotonic()
        time.sleep(execution["dwell_s"])
        log_event(log, "grasp_dwell", requested_s=execution["dwell_s"], actual_s=time.monotonic() - started)
        move(bus, targets["lift"], check_z, log, "lift", execution)
        move(bus, observation, check_z, log, "return_observation", execution)
        move_gripper(bus, robot["gripper_open"], log, "gripper_open_at_observation", execution["gripper_tolerance"])
        log_event(log, "complete")
        print(f"GRASP_OK log={log}")
    except BaseException as exc:
        log_event(log, "error", error_type=type(exc).__name__, error=str(exc))
        if bus.is_connected:
            bus.sync_write("Goal_Velocity", dict.fromkeys(ARM, 0), normalize=False, num_retry=1)
            hold_current(bus)
            log_event(log, "hold_current")
        raise
    finally:
        if bus.is_connected:
            if original_speed is not None:
                bus.sync_write("Goal_Velocity", original_speed, normalize=False, num_retry=1)
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
