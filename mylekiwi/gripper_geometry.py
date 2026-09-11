from __future__ import annotations

from pathlib import Path

import numpy as np

from .kinematics import SO101ForwardKinematics


class GripperFloorGuard:
    def __init__(self, offsets_deg, urdf: Path, hull: Path, table_z_m: float, margin_m: float):
        self.kinematics = SO101ForwardKinematics(urdf, "gripper_link", offsets_deg)
        self.vertices = np.load(hull)["vertices_gripper_link"]
        self.floor_z = float(table_z_m + margin_m)

    def min_z(self, motor_deg) -> float:
        pose = self.kinematics.forward(np.asarray(motor_deg, float))
        return float(np.min(self.vertices @ pose[2, :3] + pose[2, 3]))
