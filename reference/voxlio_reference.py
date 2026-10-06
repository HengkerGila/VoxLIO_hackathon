"""VoxLIO golden model: algorithmic truth for v0.1.

run() is a point-by-point transcription of the top-level pseudocode. With
dtype=np.float32 every operation is a single-precision IEEE operation in the
same order as the C++ code, so the result is expected to match the hardware
float baseline bit for bit. With dtype=np.float64 the same float32 inputs go
through double-precision arithmetic, which measures the rounding error of the
single-precision pipeline.
"""

from dataclasses import dataclass, field

import numpy as np

from . import config as cfg
from .geometry import (TRI, flatten_index, grid_constants, jacobian,
                       plane_residual, transform_point, voxel_index)

ERROR_BITS = cfg.STATUS_BAD_POSE | cfg.STATUS_POINT_OVERFLOW


@dataclass
class VoxLIOResult:
    H: np.ndarray = field(default_factory=lambda: np.zeros(21))
    g: np.ndarray = field(default_factory=lambda: np.zeros(6))
    cost: float = 0.0
    inlier_count: int = 0
    processed_count: int = 0
    rejected_count: int = 0
    status: int = 0


def run(scan, num_points, vmap, R, t, dtype=np.float32, stats=None):
    """Run the scan-to-map front-end.

    scan        (N, 3) points in the LiDAR frame, N >= min(num_points, MAX_POINTS)
    num_points  number of points the host asks for
    vmap        VoxelMap
    R, t        predicted pose, p_map = R p + t
    stats       optional dict; filled with diagnostics that the hardware does
                not output (rejection breakdown, candidate counts, value ranges)
    """
    f = dtype
    zero = f(0)
    inf = f(np.inf)
    threshold = f(cfg.RESIDUAL_THRESHOLD)
    mins, inv_voxel_size = grid_constants(f)
    radius = range(-cfg.NEIGHBOR_RADIUS, cfg.NEIGHBOR_RADIUS + 1)

    R = [f(v) for v in np.asarray(R, dtype=np.float32).reshape(9)]
    t = [f(v) for v in np.asarray(t, dtype=np.float32).reshape(3)]

    H = [zero] * 21
    g = [zero] * 6
    cost = zero
    inliers = processed = rejected = 0
    out_of_grid = False

    diag = dict(out_of_grid=0, no_candidate=0, over_threshold=0,
                candidates_in_grid=0, candidates_valid=0,
                max_abs_point=0.0, max_abs_residual=0.0, max_abs_jacobian=0.0)

    status = 0
    if num_points > cfg.MAX_POINTS:
        status |= cfg.STATUS_POINT_OVERFLOW
    if not np.all(np.isfinite(R + t)):
        status |= cfg.STATUS_BAD_POSE

    if status == 0:
        points = np.asarray(scan, dtype=np.float32).reshape(-1, 3)
        if len(points) < num_points:
            raise ValueError(f"scan holds {len(points)} points, num_points is {num_points}")
        points = points[:num_points].astype(f)
        desc = np.asarray(vmap.desc, dtype=np.float32).astype(f)
        # A descriptor is usable when its valid flag is set and its normal is
        # not the zero vector.
        usable = (np.asarray(vmap.valid) != 0) & np.any(desc[:, 3:] != 0, axis=1)
        voxels = [tuple(desc[k]) if usable[k] else None for k in range(cfg.NUM_VOXELS)]

        with np.errstate(all="ignore"):
            for q in points:
                processed += 1

                p = transform_point(R, t, tuple(q))

                centre = voxel_index(p, mins, inv_voxel_size)
                if centre is None:
                    rejected += 1
                    out_of_grid = True
                    diag["out_of_grid"] += 1
                    continue
                ix, iy, iz = centre

                best_abs = inf
                best_r = zero
                best_normal = None

                for dz in radius:
                    for dy in radius:
                        for dx in radius:
                            cx, cy, cz = ix + dx, iy + dy, iz + dz
                            if not (0 <= cx < cfg.NX and 0 <= cy < cfg.NY and 0 <= cz < cfg.NZ):
                                continue
                            diag["candidates_in_grid"] += 1

                            voxel = voxels[flatten_index(cx, cy, cz)]
                            if voxel is None:
                                continue
                            diag["candidates_valid"] += 1

                            r = plane_residual(p, voxel[:3], voxel[3:])
                            if abs(r) < best_abs:
                                best_abs = abs(r)
                                best_r = r
                                best_normal = voxel[3:]

                if best_normal is None:
                    rejected += 1
                    diag["no_candidate"] += 1
                    continue

                if not abs(best_r) < threshold:
                    rejected += 1
                    diag["over_threshold"] += 1
                    continue

                J = jacobian(p, best_normal)

                for k, (i, j) in enumerate(TRI):
                    H[k] = H[k] + J[i] * J[j]
                for i in range(6):
                    g[i] = g[i] + J[i] * best_r
                cost = cost + best_r * best_r
                inliers += 1

                if stats is not None:
                    diag["max_abs_point"] = max(diag["max_abs_point"], *(abs(float(v)) for v in p))
                    diag["max_abs_residual"] = max(diag["max_abs_residual"], abs(float(best_r)))
                    diag["max_abs_jacobian"] = max(diag["max_abs_jacobian"], *(abs(float(v)) for v in J))

    if out_of_grid:
        status |= cfg.STATUS_OOB_PREVENTED
    if inliers == 0:
        status |= cfg.STATUS_ZERO_INLIERS
    if status & ERROR_BITS == 0 and inliers > 0:
        status |= cfg.STATUS_SUCCESS

    if stats is not None:
        stats.update(diag)

    return VoxLIOResult(
        H=np.array(H, dtype=np.float64),
        g=np.array(g, dtype=np.float64),
        cost=float(cost),
        inlier_count=inliers,
        processed_count=processed,
        rejected_count=rejected,
        status=status,
    )


# ---------------------------------------------------------------------------
# Result files: "key [index] value" lines, floats as C99 hex so they round-trip
# exactly between Python and the C++ testbenches.
# ---------------------------------------------------------------------------


def write_result(path, result):
    lines = [
        f"inlier_count {result.inlier_count}",
        f"processed_count {result.processed_count}",
        f"rejected_count {result.rejected_count}",
        f"status {result.status}",
    ]
    lines += [f"H {k} {float(v).hex()}" for k, v in enumerate(result.H)]
    lines += [f"g {k} {float(v).hex()}" for k, v in enumerate(result.g)]
    lines.append(f"cost {float(result.cost).hex()}")
    path.write_text("\n".join(lines) + "\n")


def read_result(path):
    result = VoxLIOResult()
    for line in path.read_text().splitlines():
        tokens = line.split()
        if not tokens:
            continue
        key = tokens[0]
        if key == "H":
            result.H[int(tokens[1])] = float.fromhex(tokens[2])
        elif key == "g":
            result.g[int(tokens[1])] = float.fromhex(tokens[2])
        elif key == "cost":
            result.cost = float.fromhex(tokens[1])
        else:
            setattr(result, key, int(tokens[1]))
    return result
