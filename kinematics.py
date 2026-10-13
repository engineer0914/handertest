"""Standard Spong DH. Internal translation unit: mm."""
import csv
import json
import re
from pathlib import Path

import numpy as np


def check_transform(matrix, name="transform"):
    T = np.asarray(matrix, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError(f"{name}: expected a finite 4x4 matrix")
    if not np.allclose(
        T[3], [0, 0, 0, 1], atol=1e-8, rtol=0
    ):
        raise ValueError(f"{name}: last row must be [0, 0, 0, 1]")
    R = T[:3, :3]
    if not np.allclose(
        R.T @ R, np.eye(3), atol=1e-5, rtol=0
    ):
        raise ValueError(f"{name}: rotation is not orthonormal")
    if not np.isclose(
        np.linalg.det(R), 1.0, atol=1e-5, rtol=0
    ):
        raise ValueError(f"{name}: rotation determinant must be +1")
    return T.copy()


def load_transform(filename, unit):
    """T_A_B: frame B의 점 좌표를 frame A로 변환."""
    with Path(filename).open(encoding="utf-8-sig") as f:
        T = check_transform(json.load(f), str(filename))
    if unit not in ("m", "mm"):
        raise ValueError("Translation unit must be m or mm")
    T[:3, 3] *= 1000.0 if unit == "m" else 1.0
    return T


def inverse_transform(T):
    T = check_transform(T)
    result = np.eye(4)
    result[:3, :3] = T[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ T[:3, 3]
    return result


def from_xyz_rpy(x, y, z, rx, ry, rz):
    """XYZ: mm. RPY: degrees. R = Rz(rz) @ Ry(ry) @ Rx(rx)."""
    rx, ry, rz = np.deg2rad([rx, ry, rz])
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = [x, y, z]
    return check_transform(T)


def dh_transform(theta_deg, d, a, alpha_deg):
    """Standard DH: Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)."""
    theta, alpha = np.deg2rad([theta_deg, alpha_deg])
    c, s = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [c, -s * ca, s * sa, a * c],
        [s, c * ca, -c * sa, a * s],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])


class RobotArm:
    def __init__(self, dh_param_file):
        self.rows = []
        joints = []

        with Path(dh_param_file).open(
            encoding="utf-8-sig", newline=""
        ) as f:
            reader = csv.DictReader(f)
            required = {
                "link_id", "theta_var", "theta_offset_deg",
                "d", "a", "alpha_deg",
            }
            if not required.issubset(reader.fieldnames or []):
                raise ValueError(
                    f"DH CSV requires columns: {sorted(required)}"
                )
            for row in reader:
                if not any(row.values()):
                    continue

                link_id = int(row["link_id"])
                if link_id != len(self.rows) + 1:
                    raise ValueError(
                        "DH rows must have ordered link_id = 1, 2, ..."
                    )

                variable = row["theta_var"].strip()
                if variable == "0":
                    joint = None
                else:
                    match = re.fullmatch(r"theta([1-6])", variable)
                    if not match:
                        raise ValueError(
                            f"link {link_id}: invalid theta_var {variable!r}"
                        )
                    joint = int(match.group(1)) - 1
                    joints.append(joint)

                values = [
                    float(row[key])
                    for key in ("theta_offset_deg", "d", "a", "alpha_deg")
                ]
                if not np.isfinite(values).all():
                    raise ValueError(
                        f"link {link_id}: non-finite DH value"
                    )
                self.rows.append((joint, *values))

        if sorted(joints) != list(range(6)):
            raise ValueError(
                "DH CSV must contain theta1 through theta6 exactly once"
            )

    def get_all_link_poses(self, joint_angles_deg):
        q = np.asarray(joint_angles_deg, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError(
                "Expected six finite joint angles in degrees"
            )

        poses = [np.eye(4)]
        for joint, offset, d, a, alpha in self.rows:
            theta = offset + (0.0 if joint is None else q[joint])
            poses.append(
                poses[-1] @ dh_transform(theta, d, a, alpha)
            )
        return poses
