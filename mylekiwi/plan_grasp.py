"""Wrist RGB -> DINO + SAM2 + Depth Anything + AnyGrasp -> SO-101 plan."""

import json
import math
import os
import subprocess
import sys
import time
from collections import Counter
from datetime import datetime

import cv2
import numpy as np
from PIL import Image
from lerobot.model.kinematics import RobotKinematics
from scipy.spatial.transform import Rotation

from .config import ROOT, load_config, project_path
from .gripper_geometry import GripperFloorGuard
from .hardware import ARM

CFG = load_config()
JOINTS = [name.removeprefix("arm_") for name in ARM]


def load_models():
    import torch
    from transformers import (
        AutoImageProcessor,
        AutoModelForDepthEstimation,
        AutoModelForZeroShotObjectDetection,
        AutoProcessor,
        Sam2Model,
        Sam2Processor,
    )

    device = "cuda" if torch.cuda.is_available() else "cpu"
    paths = CFG["paths"]
    dino_processor = AutoProcessor.from_pretrained(project_path(paths["dino"]), local_files_only=True)
    dino = AutoModelForZeroShotObjectDetection.from_pretrained(
        project_path(paths["dino"]), local_files_only=True
    ).to(device).eval()
    depth_processor = AutoImageProcessor.from_pretrained(project_path(paths["depth"]), local_files_only=True)
    depth = AutoModelForDepthEstimation.from_pretrained(
        project_path(paths["depth"]), local_files_only=True
    ).to(device).eval()
    sam_processor = Sam2Processor.from_pretrained(project_path(paths["sam2"]), local_files_only=True)
    sam = Sam2Model.from_pretrained(project_path(paths["sam2"]), local_files_only=True).to(device).eval()
    return device, dino_processor, dino, sam_processor, sam, depth_processor, depth


def detect_and_depth(rgb: np.ndarray, models):
    import torch

    device, dino_p, dino, sam_p, sam, depth_p, depth = models
    cfg = CFG["perception"]
    inputs = dino_p(images=rgb, text=cfg["prompt"], return_tensors="pt").to(device)
    with torch.inference_mode():
        output = dino(**inputs)
    result = dino_p.post_process_grounded_object_detection(
        output,
        inputs.input_ids,
        threshold=cfg["box_threshold"],
        text_threshold=cfg["text_threshold"],
        target_sizes=[rgb.shape[:2]],
    )[0]
    if not len(result["boxes"]):
        raise RuntimeError(f"GroundingDINO did not find {cfg['prompt']!r}")
    on_table = ((result["boxes"][:, 1] + result["boxes"][:, 3]) / 2) > rgb.shape[0] / 2
    if not on_table.any():
        raise RuntimeError("GroundingDINO found no object in the lower table region")
    index = int(result["scores"].masked_fill(~on_table, -1).argmax())
    box = result["boxes"][index].cpu().numpy()
    score = float(result["scores"][index])

    inputs = sam_p(images=rgb, input_boxes=[[box.tolist()]], return_tensors="pt").to(device)
    with torch.inference_mode():
        output = sam(**inputs, multimask_output=True)
    best = int(output.iou_scores[0, 0].argmax())
    mask = sam_p.post_process_masks(output.pred_masks.cpu(), inputs["original_sizes"].cpu())[0][0, best].numpy()
    if not mask.any():
        raise RuntimeError("SAM2 returned an empty mask")

    inputs = depth_p(images=rgb, return_tensors="pt").to(device)
    with torch.inference_mode():
        relative = depth(**inputs).predicted_depth
    relative = torch.nn.functional.interpolate(
        relative[:, None], rgb.shape[:2], mode="bicubic", align_corners=False
    )[0, 0].cpu().numpy()
    return box, score, mask, relative


def depth_to_cloud(rgb: np.ndarray, relative: np.ndarray, mask: np.ndarray, camera: np.ndarray):
    calibration = json.loads(project_path(CFG["paths"]["depth_calibration"]).read_text())
    perception = CFG["perception"]
    if calibration["rmse_m"] > perception["depth_rmse_max_m"]:
        raise RuntimeError("metric-depth RMSE is above the configured limit")
    if calibration["max_abs_error_m"] > perception["depth_error_max_m"]:
        raise RuntimeError("metric-depth max error is above the configured limit")
    depth = calibration["A"] * relative + calibration["B"]
    y, x = np.mgrid[: depth.shape[0], : depth.shape[1]]
    valid = mask & (depth > perception["depth_min_m"]) & (depth < perception["depth_max_m"])
    z = depth[valid]
    points = np.column_stack((
        (x[valid] - camera[0, 2]) * z / camera[0, 0],
        (y[valid] - camera[1, 2]) * z / camera[1, 1],
        z,
    )).astype(np.float32)
    if len(points) < 100:
        raise RuntimeError("masked point cloud is too small")
    return points, rgb[valid].astype(np.float32) / 255, depth


def get_grasps(points, colors, mask, depth, camera, output):
    perception, anygrasp = CFG["perception"], CFG["anygrasp"]
    cloud, result = (output / "cloud.npz").resolve(), (output / "anygrasp.json").resolve()
    np.savez_compressed(
        cloud,
        points=points,
        colors=colors,
        lims=[-0.4, 0.4, -0.4, 0.4, perception["depth_min_m"], perception["depth_max_m"]],
    )
    python = os.environ.get("ANYGRASP_PYTHON", sys.executable)
    root = os.environ.get("ANYGRASP_ROOT")
    if not root:
        raise RuntimeError("set ANYGRASP_ROOT to the licensed AnyGrasp grasp_detection directory")
    worker = project_path(CFG["paths"]["anygrasp_worker"])
    subprocess.run(
        [
            python,
            str(worker),
            "--anygrasp-root", root,
            "--input-npz", str(cloud),
            "--output-json", str(result),
            "--top-k", str(anygrasp["top_k"]),
            "--max-gripper-width", str(anygrasp["max_width_m"]),
            "--collision-detection",
        ],
        check=True,
    )
    cloud.unlink(missing_ok=True)
    grasps = json.loads(result.read_text())["grasps"]
    ring = cv2.dilate(mask.astype(np.uint8), np.ones((31, 31), np.uint8)).astype(bool) & ~mask
    ring &= (depth > perception["depth_min_m"]) & (depth < perception["depth_max_m"])
    if not ring.any():
        raise RuntimeError("cannot estimate table depth around the SAM2 mask")
    table_depth = float(np.median(depth[ring]))
    passed = []
    for index, grasp in enumerate(grasps):
        if not 0 < grasp["width"] <= anygrasp["max_width_m"] + 1e-6:
            continue
        if grasp["translation"][2] >= table_depth - anygrasp["table_depth_margin_m"]:
            continue
        uvw = camera @ np.asarray(grasp["translation"])
        u, v = np.rint(uvw[:2] / uvw[2]).astype(int)
        if 0 <= v < mask.shape[0] and 0 <= u < mask.shape[1] and mask[v, u]:
            passed.append(index)
    if not passed:
        raise RuntimeError("no AnyGrasp candidate passed mask, width and table filters")
    return grasps, passed, table_depth


def grasp_to_ik(grasps, indices, motor_deg):
    """Use the best AnyGrasp center, but force a vertical tool orientation."""
    cfg = CFG["grasp"]
    handeye = json.loads(project_path(CFG["paths"]["handeye"]).read_text())
    offsets = np.asarray([handeye["joint_offsets_deg"][name] for name in ARM])
    calibration = json.loads(project_path(CFG["paths"]["calibration"]).read_text())
    motor_lo, motor_hi = [], []
    for name in ARM:
        low, high = calibration[name]["range_min"], calibration[name]["range_max"]
        margin, midpoint = math.ceil((high - low) * 0.05), (low + high) / 2
        motor_lo.append((low + margin - midpoint) * 360 / 4095)
        motor_hi.append((high - margin - midpoint) * 360 / 4095)
    q0 = np.asarray(motor_deg) - offsets
    low = np.maximum(cfg["joint_min_deg"], np.asarray(motor_lo) - offsets)
    high = np.minimum(cfg["joint_max_deg"], np.asarray(motor_hi) - offsets)
    urdf = project_path(CFG["paths"]["urdf"])
    camera_kin = RobotKinematics(str(urdf), "gripper_link", JOINTS)
    ee_kin = RobotKinematics(str(urdf), "gripper_frame_link", JOINTS)
    tcp_kin = RobotKinematics(str(urdf), "anygrasp_tcp_link", JOINTS)
    base_camera = camera_kin.forward_kinematics(q0) @ np.asarray(handeye["T_gripper_camera"])
    guard = GripperFloorGuard(
        offsets,
        urdf,
        project_path(CFG["paths"]["gripper_hull"]),
        cfg["table_z_m"],
        cfg["floor_margin_m"],
    )

    grasp_index = max(indices, key=lambda index: grasps[index]["score"])
    grasp = grasps[grasp_index]
    raw_center = (base_camera @ [*np.asarray(grasp["translation"]), 1.0])[:3]
    center = raw_center.copy()
    center[2] = cfg["table_z_m"] + cfg["tcp_height_m"]
    down_center = center.copy()
    down_center[:2] += [cfg["x_offset_m"], cfg["y_offset_m"]]
    vertical = Rotation.from_euler("y", 90, degrees=True).as_matrix()
    candidates = []

    for yaw in range(0, 360, int(cfg["yaw_step_deg"])):
        down = np.eye(4)
        down[:3, :3] = vertical @ Rotation.from_euler("x", yaw, degrees=True).as_matrix()
        down[:3, 3] = down_center
        above, grasp_pose, lift = down.copy(), down.copy(), down.copy()
        above[2, 3] += cfg["approach_clearance_m"]
        grasp_pose[:3, 3] = center
        lift[:] = grasp_pose
        lift[2, 3] += cfg["lift_m"]
        poses = {"above": above, "down": down, "grasp": grasp_pose, "lift": lift}
        q, targets, errors, margins = q0.copy(), {}, {}, []
        for stage, pose in poses.items():
            for _ in range(cfg["ik_iterations"]):
                q = np.clip(tcp_kin.inverse_kinematics(q, pose, 1.0, cfg["orientation_weight"]), low, high)
            actual = tcp_kin.forward_kinematics(q)
            errors[stage] = {
                "position_mm": float(np.linalg.norm(actual[:3, 3] - pose[:3, 3]) * 1000),
                "rotation_deg": float(np.degrees(Rotation.from_matrix(pose[:3, :3] @ actual[:3, :3].T).magnitude())),
                "approach_axis_deg": float(np.degrees(np.arccos(np.clip(pose[:3, 0] @ actual[:3, 0], -1, 1)))),
                "jaw_axis_deg": float(np.degrees(np.arccos(np.clip(pose[:3, 1] @ actual[:3, 1], -1, 1)))),
            }
            margins.append(np.minimum(q - low, high - q))
            targets[stage] = q + offsets
        path = [np.asarray(motor_deg), *targets.values(), np.asarray(motor_deg)]
        path_min_z = min(
            guard.min_z(start + t * (end - start))
            for start, end in zip(path, path[1:], strict=False)
            for t in np.linspace(0, 1, 101)
        )
        maximum = {key: max(item[key] for item in errors.values()) for key in errors["grasp"]}
        wrist_roll = max(abs(value[4]) for value in targets.values())
        joint_move = max(np.max(np.abs(end - start)) for start, end in zip(path, path[1:]))
        checks = (
            (maximum["position_mm"] > cfg["position_error_mm"], "position_error"),
            (maximum["approach_axis_deg"] > cfg["approach_error_deg"], "approach_axis_error"),
            (maximum["jaw_axis_deg"] > cfg["jaw_error_deg"], "jaw_axis_error"),
            (wrist_roll > cfg["wrist_roll_limit_deg"], "wrist_roll_limit"),
            (joint_move > cfg["max_joint_move_deg"], "joint_motion_limit"),
            (path_min_z < guard.floor_z + cfg["floor_path_reserve_m"], "floor_path_reserve"),
        )
        reasons = [name for failed, name in checks if failed]
        candidates.append({
            "ok": not reasons,
            "reason": ",".join(reasons) or None,
            "grasp_index": grasp_index,
            "anygrasp_score": grasp["score"],
            "raw_center_base_m": raw_center.tolist(),
            "down_center_base_m": down_center.tolist(),
            "center_base_m": center.tolist(),
            "yaw_deg": yaw,
            "targets": {stage: value.tolist() for stage, value in targets.items()},
            "poses_base": {stage: value.tolist() for stage, value in poses.items()},
            "ik_error": errors,
            "minimum_joint_limit_margin_deg": float(min(value.min() for value in margins)),
            "maximum_wrist_roll_deg": float(wrist_roll),
            "maximum_joint_motion_deg": float(joint_move),
            "full_gripper_path_min_z_m": float(path_min_z),
            "floor_guard_z_m": guard.floor_z,
            "floor_path_reserve_margin_m": float(path_min_z - guard.floor_z - cfg["floor_path_reserve_m"]),
        })

    valid = [candidate for candidate in candidates if candidate["ok"]]
    best = min(
        valid,
        key=lambda item: (
            item["maximum_joint_motion_deg"],
            item["maximum_wrist_roll_deg"],
            max(error["position_mm"] for error in item["ik_error"].values()),
        ),
        default=None,
    )
    closest = min(
        candidates,
        key=lambda item: (
            len(item["reason"].split(",")) if item["reason"] else 0,
            max(error["position_mm"] for error in item["ik_error"].values()),
        ),
        default=None,
    )
    summary = {
        "evaluated": len(candidates),
        "valid": len(valid),
        "grasp_index": grasp_index,
        "anygrasp_score": grasp["score"],
        "raw_center_base_m": raw_center.tolist(),
        "down_center_base_m": down_center.tolist(),
        "center_base_m": center.tolist(),
        "rejections": dict(Counter(reason for item in candidates for reason in (item["reason"] or "").split(",") if reason)),
        "closest_rejected": None if best else closest,
    }
    t_g_e = np.linalg.inv(tcp_kin.forward_kinematics(q0)) @ ee_kin.forward_kinematics(q0)
    return best, summary, t_g_e


def main() -> None:
    started = time.perf_counter()
    capture_path = project_path(CFG["paths"]["capture"])
    if not capture_path.is_file():
        raise FileNotFoundError(f"missing wrist RGB: {capture_path}")
    capture = json.loads(Image.open(capture_path).info["lekiwi_capture"])
    expected = CFG["robot"]["observation_joint_deg"]
    if np.max(np.abs(np.asarray(capture["joint_deg"]) - expected)) > CFG["robot"]["capture_pose_tolerance_deg"]:
        raise RuntimeError(f"capture is not from observation pose {expected}")
    bgr = cv2.imread(str(capture_path))
    if bgr is None:
        raise RuntimeError(f"cannot read wrist RGB: {capture_path}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    intrinsics = json.loads(project_path(CFG["paths"]["intrinsics"]).read_text())
    camera = np.asarray(intrinsics["camera_matrix"], float)
    source_w, source_h = intrinsics["image_size_px"]
    camera[0] *= rgb.shape[1] / source_w
    camera[1] *= rgb.shape[0] / source_h

    output_root = project_path(CFG["paths"]["plans"])
    output = output_root / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    latest = output_root / "latest"
    output.mkdir(parents=True)
    latest.mkdir(parents=True, exist_ok=True)
    box, dino_score, mask, relative = detect_and_depth(rgb, load_models())
    points, colors, depth = depth_to_cloud(rgb, relative, mask, camera)
    grasps, passed, table_depth = get_grasps(points, colors, mask, depth, camera, output)
    best, summary, t_g_e = grasp_to_ik(grasps, passed, capture["joint_deg"])

    x0, y0, x1, y1 = np.rint(box).astype(int)
    dino_view = bgr.copy()
    cv2.rectangle(dino_view, (x0, y0), (x1, y1), (0, 255, 0), 3)
    cv2.putText(dino_view, f"black object {dino_score:.3f}", (x0, max(24, y0 - 8)), 0, 0.7, (0, 255, 0), 2)
    sam_view = bgr.copy()
    sam_view[mask] = (0.45 * sam_view[mask] + 0.55 * np.array([0, 255, 0])).astype(np.uint8)
    low, high = np.percentile(depth, [2, 98])
    depth_view = cv2.applyColorMap(
        255 - np.clip((depth - low) / (high - low) * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO
    )
    selected = max(passed, key=lambda index: grasps[index]["score"])
    grasp_view = bgr.copy()
    for index, grasp in enumerate(grasps):
        center = np.asarray(grasp["translation"])
        if center[2] <= 0:
            continue
        pixel = camera @ center
        pixel = np.rint(pixel[:2] / pixel[2]).astype(int)
        color = (0, 255, 0) if index == selected else (128, 128, 128)
        cv2.circle(grasp_view, tuple(pixel), 8 if index == selected else 2, color, 2 if index == selected else -1)
    images = {
        "rgb.png": bgr,
        "groundingdino.png": dino_view,
        "sam2.png": sam_view,
        "depth.png": depth_view,
        "anygrasp.png": grasp_view,
    }
    images["pipeline.png"] = cv2.hconcat([cv2.resize(image, (384, 216)) for image in images.values()])
    for name, image in images.items():
        cv2.imwrite(str(output / name), image)
        cv2.imwrite(str(latest / name), image)

    grasp_cfg = CFG["grasp"]
    plan = {
        "planner_revision": 6,
        "execution_mode": "offset_descent_then_center_grasp" if best else "no_safe_offset_descent_then_center_grasp",
        "prompt": CFG["perception"]["prompt"],
        "image": str(capture_path),
        "observation_joint_deg": capture["joint_deg"],
        "dino_score": dino_score,
        "box": box.tolist(),
        "mask_pixels": int(mask.sum()),
        "point_count": len(points),
        "table_depth_m": table_depth,
        "table_z_m": grasp_cfg["table_z_m"],
        "descent_xy_offset_base_m": [grasp_cfg["x_offset_m"], grasp_cfg["y_offset_m"]],
        "T_G_E": t_g_e.tolist(),
        "approach": "vertical at offset Base XY, then cancel offset to the AnyGrasp center at grasp Z",
        "trajectory": {
            "stages": ["above", "down", "grasp", "lift"],
            "approach_clearance_m": grasp_cfg["approach_clearance_m"],
            "lift_m": grasp_cfg["lift_m"],
            "yaw_step_deg": grasp_cfg["yaw_step_deg"],
        },
        "limits": {key: grasp_cfg[key] for key in (
            "position_error_mm", "approach_error_deg", "jaw_error_deg", "wrist_roll_limit_deg",
            "max_joint_move_deg", "floor_margin_m", "floor_path_reserve_m",
        )},
        "candidate_summary": summary,
        "grasp_execution": best or {"ok": False, "reason": "no safe offset-descent then center grasp"},
        "timing_s": {"total": time.perf_counter() - started},
        "visualizations": {name: str(output / name) for name in images},
    }
    text = json.dumps(plan, indent=2) + "\n"
    (output / "plan.json").write_text(text)
    (latest / "plan.json").write_text(text)
    print(json.dumps({
        "plan": str(output / "plan.json"),
        "mode": plan["execution_mode"],
        "dino_score": dino_score,
        "mask_pixels": plan["mask_pixels"],
        "points": len(points),
        "candidates": summary["evaluated"],
        "valid": summary["valid"],
        "selected": None if not best else {
            key: best[key] for key in (
                "grasp_index", "anygrasp_score", "yaw_deg", "center_base_m", "targets", "ik_error",
                "maximum_wrist_roll_deg", "minimum_joint_limit_margin_deg", "full_gripper_path_min_z_m",
            )
        },
    }, indent=2))


if __name__ == "__main__":
    main()
