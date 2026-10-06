"""Geometry shared by the golden model, the map builder and the host solver.

Two groups of functions live here.

Accelerator math (transform_point ... jacobian) works on scalars of one numpy
float type and evaluates every expression in the same order as the C++ code,
so a float32 run reproduces the hardware float baseline operation for
operation. Callers pass scalars that already have the wanted dtype.

Host math (rotations, pose update, 6 x 6 solve) is ordinary float64.

Pose convention, used everywhere:
    p_map = R @ p_lidar + t
    state ordering   delta = [wx, wy, wz, tx, ty, tz]
    perturbation     p_map' = Exp(w) @ p_map + tau        (left / map frame)
    update           R <- Exp(w) @ R,  t <- Exp(w) @ t + tau
"""

import numpy as np

from . import config as cfg

# Packed upper triangle of H: H[k] = H(i, j), i <= j,
# k = i + j * (j + 1) / 2.
TRI = [(i, j) for j in range(6) for i in range(j + 1)]


# ---------------------------------------------------------------------------
# Accelerator math
# ---------------------------------------------------------------------------


def grid_constants(dtype):
    """(map minimum per axis, 1 / voxel size) as scalars of `dtype`."""
    mins = tuple(dtype(v) for v in cfg.MAP_MIN)
    inv_voxel_size = dtype(1.0) / dtype(cfg.VOXEL_SIZE)
    return mins, inv_voxel_size


def transform_point(R, t, q):
    """R (9, row-major), t (3), q (3) -> p = R q + t."""
    x, y, z = q
    return (
        R[0] * x + R[1] * y + R[2] * z + t[0],
        R[3] * x + R[4] * y + R[5] * z + t[1],
        R[6] * x + R[7] * y + R[8] * z + t[2],
    )


def voxel_index(p, mins, inv_voxel_size):
    """Voxel coordinate (ix, iy, iz) of p, or None when p is outside the grid.

    The range test is applied to the scaled grid coordinate, the value that is
    truncated, so the returned indices are in range by construction. NaN and
    Inf fail the test.
    """
    index = []
    for v, v_min, n in zip(p, mins, cfg.GRID_DIMS):
        f = (v - v_min) * inv_voxel_size
        if not (f >= 0 and f < n):
            return None
        index.append(int(f))
    return tuple(index)


def flatten_index(ix, iy, iz):
    return (iz * cfg.NY + iy) * cfg.NX + ix


def plane_residual(p, c, n):
    """Signed point-to-plane distance r = n . (p - c)."""
    return n[0] * (p[0] - c[0]) + n[1] * (p[1] - c[1]) + n[2] * (p[2] - c[2])


def jacobian(p, n):
    """J = [p x n, n]."""
    return (
        p[1] * n[2] - p[2] * n[1],
        p[2] * n[0] - p[0] * n[2],
        p[0] * n[1] - p[1] * n[0],
        n[0],
        n[1],
        n[2],
    )


# ---------------------------------------------------------------------------
# Host math
# ---------------------------------------------------------------------------


def rpy_to_matrix(roll, pitch, yaw):
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll), angles in radians."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def so3_exp(w):
    """Rotation matrix for the rotation vector w (Rodrigues)."""
    w = np.asarray(w, dtype=np.float64)
    theta = np.linalg.norm(w)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    if theta < 1e-12:
        return np.eye(3) + K
    return (
        np.eye(3)
        + (np.sin(theta) / theta) * K
        + ((1.0 - np.cos(theta)) / theta**2) * (K @ K)
    )


def apply_delta(R, t, delta):
    """Left-multiplicative pose update: (Exp(w), tau) composed with (R, t)."""
    delta = np.asarray(delta, dtype=np.float64)
    dR = so3_exp(delta[:3])
    return dR @ R, dR @ t + delta[3:]


def pose_error(R_a, t_a, R_b, t_b):
    """(rotation angle in rad, translation distance in m) between two poses."""
    cos_angle = (np.trace(R_b @ R_a.T) - 1.0) / 2.0
    angle = float(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
    return angle, float(np.linalg.norm(np.asarray(t_b) - np.asarray(t_a)))


def unpack_h(h_packed):
    """21 packed values -> symmetric 6 x 6 float64 matrix."""
    H = np.zeros((6, 6))
    for k, (i, j) in enumerate(TRI):
        H[i, j] = H[j, i] = float(h_packed[k])
    return H


class SolveError(Exception):
    """The accelerator result failed a host-side sanity check."""


def solve_update(h_packed, g, inlier_count, min_inliers=50, max_condition=1e8,
                 max_rotation=0.2, max_translation=1.0):
    """Solve H delta = -g with the host-side checks."""
    H = unpack_h(h_packed)
    g = np.asarray(g, dtype=np.float64)
    if inlier_count < min_inliers:
        raise SolveError(f"only {inlier_count} inliers (minimum {min_inliers})")
    if not (np.all(np.isfinite(H)) and np.all(np.isfinite(g))):
        raise SolveError("H or g is not finite")
    condition = np.linalg.cond(H)
    if not condition < max_condition:
        raise SolveError(f"H is ill-conditioned (cond = {condition:.3g})")
    delta = np.linalg.solve(H, -g)
    if np.linalg.norm(delta[:3]) > max_rotation or np.linalg.norm(delta[3:]) > max_translation:
        raise SolveError(f"update too large: {delta}")
    return delta
