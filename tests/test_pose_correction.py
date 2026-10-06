"""Host loop on the synthetic room : the reduced H and g
must produce a correction that points toward the ground-truth pose."""

import numpy as np
import pytest

from reference import generate_synthetic as gen
from reference.geometry import SolveError, apply_delta, pose_error, solve_update
from reference.voxel_map import build_voxel_map
from reference.voxlio_reference import run

from maps import ground_map, IDENTITY


@pytest.fixture(scope="module")
def room():
    R_gt, t_gt = gen.ground_truth_pose()
    scan = gen.make_scan(R_gt, t_gt)
    vmap = build_voxel_map(gen.make_map_points())
    return scan, vmap, R_gt, t_gt


def step(scan, vmap, R, t):
    res = run(scan, len(scan), vmap, R, t)
    return solve_update(res.H, res.g, res.inlier_count)


def test_first_correction_points_toward_ground_truth(room):
    scan, vmap, R_gt, t_gt = room
    R, t = gen.predicted_pose(R_gt, t_gt)
    delta = step(scan, vmap, R, t)

    ideal = -gen.PREDICTION_ERROR
    cosine = delta @ ideal / (np.linalg.norm(delta) * np.linalg.norm(ideal))
    assert cosine > 0.98

    before = pose_error(R, t, R_gt, t_gt)
    after = pose_error(*apply_delta(R, t, delta), R_gt, t_gt)
    assert after[0] < before[0] / 4 and after[1] < before[1] / 4


def test_iterations_converge_to_ground_truth(room):
    scan, vmap, R_gt, t_gt = room
    R, t = gen.predicted_pose(R_gt, t_gt)
    errors = [pose_error(R, t, R_gt, t_gt)]
    for _ in range(5):
        R, t = apply_delta(R, t, step(scan, vmap, R, t))
        errors.append(pose_error(R, t, R_gt, t_gt))
    assert all(b[1] < a[1] for a, b in zip(errors, errors[1:]))
    assert np.rad2deg(errors[-1][0]) < 0.02 and errors[-1][1] < 0.002


def test_ground_truth_pose_is_nearly_a_fixed_point(room):
    scan, vmap, R_gt, t_gt = room
    delta = step(scan, vmap, R_gt, t_gt)
    assert np.linalg.norm(delta[:3]) < np.deg2rad(0.02) and np.linalg.norm(delta[3:]) < 0.002


def test_solver_rejects_too_few_inliers():
    with pytest.raises(SolveError, match="inliers"):
        solve_update(np.zeros(21), np.zeros(6), inlier_count=3)


def test_solver_rejects_singular_h():
    # A single plane constrains only three of the six degrees of freedom.
    points = np.array([[0.1 * i, 0.07 * j, 0.01] for i in range(-3, 4) for j in range(-3, 4)],
                      dtype=np.float32)
    res = run(points, len(points), ground_map(), *IDENTITY)
    with pytest.raises(SolveError, match="ill-conditioned"):
        solve_update(res.H, res.g, res.inlier_count, min_inliers=10)


def test_solver_rejects_non_finite_input():
    H = np.zeros(21)
    H[0] = np.nan
    with pytest.raises(SolveError, match="finite"):
        solve_update(H, np.zeros(6), inlier_count=100)
