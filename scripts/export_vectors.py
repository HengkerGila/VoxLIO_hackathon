#!/usr/bin/env python3
"""Generate the synthetic test vectors and the golden-model reference outputs.

For every case this writes, under data/synthetic/<case>/:
    scan.bin          float32 (num_points, 3), LiDAR frame
    voxel_desc.bin    float32 (NUM_VOXELS, 6): cx cy cz nx ny nz
    voxel_valid.bin   uint8   (NUM_VOXELS,)
    pose.bin          float32 (12,): predicted R row-major, then t
    expected_f32.txt  golden model, single precision (matches the hardware float baseline)
    expected_f64.txt  golden model, double precision arithmetic on the same inputs
    meta.txt          configuration, scene and diagnostics as "key value" lines
"""

import pathlib
import sys
from dataclasses import dataclass

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from reference import config as cfg                      # noqa: E402
from reference import generate_synthetic as gen          # noqa: E402
from reference.voxel_map import build_voxel_map          # noqa: E402
from reference.voxlio_reference import run, write_result  # noqa: E402

@dataclass(frozen=True)
class Case:
    scene: gen.Scene = gen.Scene()
    map_seed: int = 1
    scan_seed: int = 2
    # Error of the predicted pose: rotation vector (rad), translation (m).
    prediction_error: tuple = tuple(gen.PREDICTION_ERROR)


def _error(rot_deg, trans):
    return tuple(np.concatenate([np.deg2rad(rot_deg), trans]))


# "room" is the reference scene. In "room_face_aligned" the same
# room is moved by half a voxel so that all three planes lie exactly on voxel
# faces; it measures what the half-voxel grid-origin offset buys. The "seed"
# cases repeat "room" with other noise and other prediction errors.
CASES = {
    "room": Case(),
    "room_face_aligned": Case(scene=gen.Scene(ground_z=0.25, wall_x=4.25, wall_y=4.25)),
    "room_seed_b": Case(map_seed=11, scan_seed=12,
                        prediction_error=_error([-0.4, 0.7, -0.9], [-0.05, 0.06, -0.03])),
    "room_seed_c": Case(map_seed=21, scan_seed=22,
                        prediction_error=_error([0.9, 0.3, 0.5], [0.03, 0.08, 0.05])),
}


def export_case(name, case, out_root):
    out = out_root / name
    out.mkdir(parents=True, exist_ok=True)
    scene = case.scene

    R_gt, t_gt = gen.ground_truth_pose()
    R_pred, t_pred = gen.predicted_pose(R_gt, t_gt, np.array(case.prediction_error))
    map_points = gen.make_map_points(scene, seed=case.map_seed)
    scan = gen.make_scan(R_gt, t_gt, scene, seed=case.scan_seed)
    vmap = build_voxel_map(map_points)

    # The hardware receives float32; freeze that rounding here.
    pose = np.concatenate([R_pred.reshape(9), t_pred]).astype("<f4")
    R32, t32 = pose[:9].reshape(3, 3), pose[9:]

    scan.astype("<f4").tofile(out / "scan.bin")
    vmap.save(out)
    pose.tofile(out / "pose.bin")

    stats = {}
    res32 = run(scan, len(scan), vmap, R32, t32, np.float32, stats)
    res64 = run(scan, len(scan), vmap, R32, t32, np.float64)
    write_result(out / "expected_f32.txt", res32)
    write_result(out / "expected_f64.txt", res64)

    in_grid = len(scan) - stats["out_of_grid"]
    meta = {
        "num_points": len(scan),
        "max_points": cfg.MAX_POINTS,
        "nx": cfg.NX, "ny": cfg.NY, "nz": cfg.NZ,
        "voxel_size": cfg.VOXEL_SIZE,
        "map_x_min": cfg.MAP_MIN[0], "map_y_min": cfg.MAP_MIN[1], "map_z_min": cfg.MAP_MIN[2],
        "residual_threshold": float(np.float32(cfg.RESIDUAL_THRESHOLD)),
        "neighbor_radius": cfg.NEIGHBOR_RADIUS,
        "scene_ground_z": scene.ground_z, "scene_wall_x": scene.wall_x, "scene_wall_y": scene.wall_y,
        "map_seed": case.map_seed, "scan_seed": case.scan_seed,
        "map_points": len(map_points),
        "valid_voxels": int(vmap.valid.sum()),
        "inliers": res32.inlier_count,
        "rejected_out_of_grid": stats["out_of_grid"],
        "rejected_no_candidate": stats["no_candidate"],
        "rejected_over_threshold": stats["over_threshold"],
        "candidates_in_grid_per_point": stats["candidates_in_grid"] / in_grid,
        "candidates_valid_per_point": stats["candidates_valid"] / in_grid,
        "max_abs_point": stats["max_abs_point"],
        "max_abs_residual": stats["max_abs_residual"],
        "max_abs_jacobian": stats["max_abs_jacobian"],
        "max_abs_h": float(np.max(np.abs(res64.H))),
        "max_abs_g": float(np.max(np.abs(res64.g))),
    }
    lines = [f"{key} {value!r}" for key, value in meta.items()]
    lines.append("pose_gt " + " ".join(repr(float(v)) for v in np.concatenate([R_gt.reshape(9), t_gt])))
    lines.append("pose_pred " + " ".join(repr(float(v)) for v in pose))
    (out / "meta.txt").write_text("\n".join(lines) + "\n")

    print(f"{name}: {len(scan)} points, {meta['valid_voxels']} valid voxels, "
          f"{res32.inlier_count} inliers (f64: {res64.inlier_count}), status {res32.status}")


def main():
    out_root = ROOT / "data" / "synthetic"
    for name, case in CASES.items():
        export_case(name, case, out_root)


if __name__ == "__main__":
    main()
