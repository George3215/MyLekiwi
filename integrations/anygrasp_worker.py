#!/usr/bin/env python3
"""Isolated adapter from a masked point-cloud NPZ to AnyGrasp JSON."""

import argparse
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np


def main() -> None:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/mylekiwi-matplotlib")
    os.environ.setdefault("OMP_NUM_THREADS", "8")
    parser = argparse.ArgumentParser()
    parser.add_argument("--anygrasp-root", type=Path, required=True)
    parser.add_argument("--input-npz", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--checkpoint-path", type=Path)
    parser.add_argument("--top-k", type=int, default=20)
    parser.add_argument("--max-gripper-width", type=float, default=0.085)
    parser.add_argument("--collision-detection", action="store_true")
    args = parser.parse_args()

    root = args.anygrasp_root.resolve()
    checkpoint = args.checkpoint_path or root / "log/checkpoint_detection.tar"
    sys.path.insert(0, str(root))
    from gsnet import AnyGrasp

    cloud = np.load(args.input_npz)
    points = np.asarray(cloud["points"], np.float32)
    colors = np.asarray(cloud["colors"], np.float32)
    limits = np.asarray(cloud["lims"], float).tolist()
    if points.ndim != 2 or points.shape[1] != 3 or colors.shape != points.shape:
        raise ValueError("points and colors must both have shape (N, 3)")

    config = SimpleNamespace(
        checkpoint_path=str(checkpoint),
        max_gripper_width=min(0.1, max(0.0, args.max_gripper_width)),
        gripper_height=0.03,
        top_down_grasp=False,
        debug=False,
    )
    detector = AnyGrasp(config)
    detector.load_net()
    grasps, _ = detector.get_grasp(
        points,
        colors,
        lims=limits,
        apply_object_mask=True,
        dense_grasp=False,
        collision_detection=args.collision_detection,
    )
    raw_count = len(grasps)
    if raw_count:
        grasps = grasps.nms().sort_by_score()
    results = []
    for index in range(min(args.top_k, len(grasps))):
        grasp = grasps[index]
        results.append({
            "score": float(grasp.score),
            "translation": np.asarray(grasp.translation, float).reshape(3).tolist(),
            "rotation_matrix": np.asarray(grasp.rotation_matrix, float).reshape(3, 3).tolist(),
            "width": float(grasp.width),
            "height": float(grasp.height),
            "depth": float(grasp.depth),
            "object_id": int(getattr(grasp, "object_id", -1)),
        })
    payload = {
        "ok": True,
        "point_count": len(points),
        "raw_grasp_count": raw_count,
        "kept_grasp_count": len(grasps),
        "grasps": results,
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"AnyGrasp: points={len(points)} grasps={len(grasps)} output={args.output_json}")


if __name__ == "__main__":
    main()
