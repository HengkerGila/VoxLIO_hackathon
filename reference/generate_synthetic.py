"""Deterministic synthetic room: a ground plane and two walls.

Every structure is an axis-aligned plane, so residuals, normals and the
expected pose correction can all be reasoned about by hand. All noise comes
from seeded numpy RandomState streams, which are bit-stable across numpy
versions.
"""

from dataclasses import dataclass

import numpy as np

from .geometry import apply_delta, rpy_to_matrix


@dataclass(frozen=True)
class Scene:
    ground_z: float = 0.0
    wall_x: float = 4.0
    wall_y: float = 4.0
    lower: float = -6.0        # the planes run from here up to the walls
    wall_height: float = 1.5


def _room_points(scene, lower, spacing, noise, rng):
    """Jittered lattice on the three planes, Gaussian noise along each normal."""
    def axis(stop):
        return np.arange(lower, stop, spacing)

    def jitter(shape):
        return rng.uniform(-spacing / 2, spacing / 2, shape)

    def offset(shape):
        return rng.normal(0.0, noise, shape) if noise > 0 else np.zeros(shape)

    xs, ys = axis(scene.wall_x), axis(scene.wall_y)
    zs = scene.ground_z + np.arange(0.0, scene.wall_height, spacing)

    gx, gy = (a.ravel() for a in np.meshgrid(xs, ys))
    ground = np.column_stack([gx + jitter(gx.shape), gy + jitter(gy.shape),
                              scene.ground_z + offset(gx.shape)])

    wy, wz = (a.ravel() for a in np.meshgrid(ys, zs))
    wall_x = np.column_stack([scene.wall_x + offset(wy.shape), wy + jitter(wy.shape),
                              wz + jitter(wz.shape)])

    wx, wz = (a.ravel() for a in np.meshgrid(xs, zs))
    wall_y = np.column_stack([wx + jitter(wx.shape), scene.wall_y + offset(wx.shape),
                              wz + jitter(wz.shape)])

    return np.vstack([ground, wall_x, wall_y])


def make_map_points(scene=Scene(), spacing=0.05, noise=0.01, seed=1):
    """Dense map point cloud in the map frame, float32 (N, 3)."""
    rng = np.random.RandomState(seed)
    return _room_points(scene, scene.lower, spacing, noise, rng).astype(np.float32)


def make_scan(R_gt, t_gt, scene=Scene(), spacing=0.08, noise=0.01, n_clutter=150, seed=2):
    """Scan in the LiDAR frame as seen from the ground-truth pose, float32 (N, 3).

    Surface points are sampled on the same planes as the map but on a
    different lattice. Clutter points, uniform in a box larger than the map,
    exercise the rejection paths: outside the grid, no valid neighbour, and
    residual above the threshold. The order is shuffled.
    """
    rng = np.random.RandomState(seed)
    surface = _room_points(scene, -4.0, spacing, noise, rng)
    clutter = rng.uniform([-10.0, -10.0, -3.0], [10.0, 10.0, 3.0], (n_clutter, 3))
    points_map = np.vstack([surface, clutter])
    points_map = points_map[rng.permutation(len(points_map))]

    # p_map = R p_lidar + t  =>  p_lidar = R^T (p_map - t)
    points_lidar = (points_map - t_gt) @ R_gt
    return points_lidar.astype(np.float32)


def ground_truth_pose():
    """The reference ground-truth pose."""
    R = rpy_to_matrix(np.deg2rad(0.5), np.deg2rad(-1.0), np.deg2rad(2.0))
    t = np.array([0.20, -0.10, 0.05])
    return R, t


# Error applied to the ground truth to obtain the predicted pose:
# [rotation vector (rad), translation (m)]. The ideal accelerator correction
# is the negative of this.
PREDICTION_ERROR = np.concatenate([np.deg2rad([0.6, -0.5, 0.8]), [0.06, -0.05, 0.04]])


def predicted_pose(R_gt, t_gt, error=PREDICTION_ERROR):
    return apply_delta(R_gt, t_gt, error)
