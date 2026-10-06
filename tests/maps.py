"""Tiny hand-built voxel maps for directed tests."""

import numpy as np

from reference import config as cfg
from reference.geometry import flatten_index
from reference.voxel_map import VoxelMap

IDENTITY = (np.eye(3), np.zeros(3))

# Voxel (16, 16, 4) is centred on the map-frame origin.
CENTRE = (16, 16, 4)


def voxel_centre(ix, iy, iz):
    return tuple(m + (i + 0.5) * cfg.VOXEL_SIZE for m, i in zip(cfg.MAP_MIN, (ix, iy, iz)))


def set_plane(vmap, voxel, centroid, normal, valid=1):
    k = flatten_index(*voxel)
    vmap.desc[k] = [*centroid, *normal]
    vmap.valid[k] = valid
    return vmap


def ground_map(voxel=CENTRE, z=0.0):
    """One valid voxel holding the horizontal plane at height z."""
    cx, cy, _ = voxel_centre(*voxel)
    return set_plane(VoxelMap.empty(), voxel, (cx, cy, z), (0.0, 0.0, 1.0))
