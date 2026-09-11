"""Capture exactly one wrist RGB frame with the simultaneous arm state."""

import json
import time
from datetime import datetime

import cv2
from PIL import Image, PngImagePlugin

from .config import ROOT, load_config, project_path
from .hardware import ALL, ARM, load_calibration, make_bus


def main() -> None:
    cfg = load_config()
    robot = cfg["robot"]
    output = ROOT / "outputs/captures" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True)
    calibration = load_calibration(project_path(cfg["paths"]["calibration"]))
    bus = make_bus(robot["port"], calibration)
    camera = cv2.VideoCapture(robot["wrist_camera"])
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, robot["camera_width"])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, robot["camera_height"])
    if not camera.isOpened():
        raise RuntimeError(f"cannot open wrist camera {robot['wrist_camera']}")
    try:
        bus.connect(handshake=True)
        for _ in range(robot["camera_flush_frames"]):
            if not camera.read()[0]:
                raise RuntimeError("wrist camera read failed")
        started = time.time()
        state = bus.sync_read("Present_Position", ALL, num_retry=2)
        arm = [float(state[name]) for name in ARM]
        expected = robot["observation_joint_deg"]
        if max(abs(a - b) for a, b in zip(arm, expected, strict=True)) > robot["capture_pose_tolerance_deg"]:
            raise RuntimeError(f"arm is not at observation pose: {arm}; expected {expected}")
        if float(state["arm_gripper"]) < robot["gripper_open_min"]:
            raise RuntimeError(f"gripper is not open: {state['arm_gripper']}")
        ok, frame = camera.read()
        finished = time.time()
        if not ok:
            raise RuntimeError("wrist camera read failed")
        expected_shape = (robot["camera_height"], robot["camera_width"])
        if frame.shape[:2] != expected_shape:
            raise RuntimeError(f"unexpected frame size {frame.shape[:2]}; expected {expected_shape}")
        payload = {
            "joint_deg": arm,
            "observation_joint_deg": expected,
            "gripper_position": float(state["arm_gripper"]),
            "joint_names": ARM,
            "read_window_unix_s": [started, finished],
            "read_window_ms": (finished - started) * 1000,
            "stationary_arm_required": True,
            "motor_writes": 0,
        }
        image = output / "wrist.png"
        metadata = PngImagePlugin.PngInfo()
        metadata.add_text("lekiwi_capture", json.dumps(payload))
        Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).save(image, pnginfo=metadata)
        payload["image"] = str(image.relative_to(ROOT))
        (output / "capture.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        print(f"capture={output / 'capture.json'}")
        print(json.dumps(payload, indent=2))
        print(f"image={image}")
    finally:
        camera.release()
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
