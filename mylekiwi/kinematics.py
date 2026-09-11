"""Small URDF forward-kinematics helper used by the floor guard."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def _rpy(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cx, sx, cy, sy, cz, sz = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ])


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    c, s, d = np.cos(angle), np.sin(angle), 1 - np.cos(angle)
    return np.array([
        [c + x * x * d, x * y * d - z * s, x * z * d + y * s],
        [y * x * d + z * s, c + y * y * d, y * z * d - x * s],
        [z * x * d - y * s, z * y * d + x * s, c + z * z * d],
    ])


def _transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, translation
    return transform


class SO101ForwardKinematics:
    def __init__(self, urdf: Path, target_frame: str, offsets_deg: list[float]):
        root = ET.parse(urdf).getroot()
        joints = list(root.findall("joint"))

        def child(joint):
            node = joint.find("child")
            return None if node is None else node.get("link")

        if not any(child(joint) == target_frame for joint in joints):
            raise ValueError(f"target frame not found in URDF: {target_frame}")
        current = target_frame
        chain = []
        while current != "base_link":
            joint = next((item for item in joints if child(item) == current), None)
            if joint is None:
                raise ValueError(f"cannot trace {target_frame} to base_link")
            chain.append(joint)
            current = joint.find("parent").get("link")
        chain.reverse()
        self.chain = []
        for joint in chain:
            origin = joint.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            axis = joint.find("axis")
            axis_xyz = np.fromstring(axis.get("xyz", "0 0 1"), sep=" ") if axis is not None else np.array([0, 0, 1.0])
            self.chain.append((joint.get("name"), joint.get("type"), xyz, _rpy(rpy), axis_xyz))
        self.offsets = np.asarray(offsets_deg, float)

    def forward(self, motor_deg: np.ndarray) -> np.ndarray:
        angles = dict(zip(JOINTS, np.deg2rad(np.asarray(motor_deg) - self.offsets), strict=True))
        pose = np.eye(4)
        for name, kind, xyz, rotation, axis in self.chain:
            pose = pose @ _transform(rotation, xyz)
            if kind == "revolute" and name in angles:
                pose = pose @ _transform(_axis_angle(axis, angles[name]), np.zeros(3))
        return pose
