"""CPU-side voxel map builder.

Buckets map points into the fixed grid and fits one plane per voxel by PCA.
This is software-only in v0.1; the accelerator receives the finished
descriptors and never modifies them.
"""

from dataclasses import dataclass

import numpy as np

from . import config as cfg
from .geometry import grid_constants

MIN_POINTS = 10


@dataclass
class VoxelMap:
    desc: np.ndarray   # (NUM_VOXELS, 6) float32: cx cy cz nx ny nz
    valid: np.ndarray  # (NUM_VOXELS,) uint8
    count: np.ndarray  # (NUM_VOXELS,) int64, map points per voxel (diagnostic)

    @staticmethod
    def empty():
        return VoxelMap(
            desc=np.zeros((cfg.NUM_VOXELS, 6), dtype=np.float32),
            valid=np.zeros(cfg.NUM_VOXELS, dtype=np.uint8),
            count=np.zeros(cfg.NUM_VOXELS, dtype=np.int64),
        )

    def save(self, directory):
        self.desc.astype("<f4").tofile(directory / "voxel_desc.bin")
        self.valid.tofile(directory / "voxel_valid.bin")

    @staticmethod
    def load(directory):
        vmap = VoxelMap.empty()
        vmap.desc = np.fromfile(directory / "voxel_desc.bin", dtype="<f4").reshape(cfg.NUM_VOXELS, 6)
        vmap.valid = np.fromfile(directory / "voxel_valid.bin", dtype=np.uint8)
        return vmap


def voxel_indices(points):
    """Vectorised float32 twin of geometry.voxel_index.

    Returns (inside mask, flattened index); the index is 0 where the mask is
    False.
    """
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    mins, inv_voxel_size = grid_constants(np.float32)
    f = (pts - np.array(mins, dtype=np.float32)) * inv_voxel_size
    with np.errstate(invalid="ignore"):
        inside = np.all((f >= 0) & (f < np.array(cfg.GRID_DIMS, dtype=np.float32)), axis=1)
    idx = np.zeros(f.shape, dtype=np.int64)
    idx[inside] = f[inside].astype(np.int64)
    flat = (idx[:, 2] * cfg.NY + idx[:, 1]) * cfg.NX + idx[:, 0]
    return inside, flat


def orient_normal(normal):
    """Deterministic sign: the largest-magnitude component is positive.

    H and g do not depend on the sign (J and r flip together), so this only
    has to be repeatable. It needs no sensor origin, unlike a viewpoint-based rule,
    which is undefined when the origin lies on the plane.
    """
    k = int(np.argmax(np.abs(normal)))
    return -normal if normal[k] < 0 else normal


def build_voxel_map(points, min_points=MIN_POINTS, max_thickness_ratio=None):
    """Fit centroid + normal for every voxel holding at least `min_points`.

    max_thickness_ratio, when given, also rejects voxels whose point set is not
    plane-like: the smallest covariance eigenvalue must be at most that
    fraction of the middle one.
    """
    inside, flat = voxel_indices(points)
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)[inside]
    flat = flat[inside]

    order = np.argsort(flat, kind="stable")
    flat, pts = flat[order], pts[order]
    voxels, starts, counts = np.unique(flat, return_index=True, return_counts=True)

    vmap = VoxelMap.empty()
    for voxel, start, count in zip(voxels, starts, counts):
        vmap.count[voxel] = count
        if count < min_points:
            continue

        bucket = pts[start:start + count]
        centroid = bucket.mean(axis=0)
        centred = bucket - centroid
        covariance = centred.T @ centred / count

        eigenvalues, eigenvectors = np.linalg.eigh(covariance)  # ascending
        if max_thickness_ratio is not None and eigenvalues[0] > max_thickness_ratio * eigenvalues[1]:
            continue

        normal = eigenvectors[:, 0]
        normal = orient_normal(normal / np.linalg.norm(normal))

        vmap.desc[voxel, :3] = centroid
        vmap.desc[voxel, 3:] = normal
        vmap.valid[voxel] = 1
    return vmap
