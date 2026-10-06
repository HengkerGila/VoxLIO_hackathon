"""Directed tests of the golden model: acceptance tests 1-5 and 7
plus every defined edge case."""

import numpy as np
import pytest

from reference import config as cfg
from reference import generate_synthetic as gen
from reference.geometry import TRI, flatten_index, unpack_h
from reference.voxel_map import VoxelMap, build_voxel_map
from reference.voxlio_reference import read_result, run, write_result

from maps import CENTRE, IDENTITY, ground_map, set_plane, voxel_centre

F = np.float32
OK = cfg.STATUS_SUCCESS
ZERO = cfg.STATUS_ZERO_INLIERS
OOB = cfg.STATUS_OOB_PREVENTED


def run_points(points, vmap, pose=IDENTITY, num_points=None, dtype=F):
    scan = np.array(points, dtype=F).reshape(-1, 3)
    n = len(scan) if num_points is None else num_points
    return run(scan, n, vmap, pose[0], pose[1], dtype)


def expected_from(J, r):
    """H, g, cost for one constraint, in float32 like the model."""
    J = [F(v) for v in J]
    r = F(r)
    return ([J[i] * J[j] for i, j in TRI], [v * r for v in J], r * r)


def assert_empty(res, status, processed, rejected):
    assert res.inlier_count == 0
    assert (res.processed_count, res.rejected_count) == (processed, rejected)
    assert res.status == status
    assert not res.H.any() and not res.g.any() and res.cost == 0.0


# ---- acceptance tests ------------------------------------------------------

def test_1_single_point_known_residual():
    res = run_points([[0.1, -0.05, 0.125]], ground_map())
    H, g, cost = expected_from([-0.05, -0.1, 0.0, 0.0, 0.0, 1.0], 0.125)
    assert res.inlier_count == 1 and res.rejected_count == 0 and res.status == OK
    assert np.array_equal(res.H, np.array(H, dtype=np.float64))
    assert np.array_equal(res.g, np.array(g, dtype=np.float64))
    assert res.cost == 0.015625


def test_2_point_on_plane_has_zero_residual():
    res = run_points([[0.2, 0.1, 0.0]], ground_map())
    assert res.inlier_count == 1 and res.status == OK
    assert res.cost == 0.0 and not res.g.any()
    assert unpack_h(res.H)[5, 5] == 1.0


def test_3_point_outside_map_is_rejected():
    res = run_points([[100.0, 0.0, 0.0]], ground_map())
    assert_empty(res, ZERO | OOB, processed=1, rejected=1)


@pytest.mark.parametrize("voxel", [(0, 0, 0), (cfg.NX - 1, cfg.NY - 1, cfg.NZ - 1),
                                   (0, cfg.NY - 1, 3), (cfg.NX - 1, 0, 0)])
def test_4_boundary_voxel_is_matched(voxel):
    cx, cy, cz = voxel_centre(*voxel)
    vmap = set_plane(VoxelMap.empty(), voxel, (cx, cy, cz), (0, 0, 1))
    res = run_points([[cx, cy, cz + 0.0625]], vmap)
    assert res.inlier_count == 1 and res.status == OK
    assert res.cost == 0.0625 ** 2


@pytest.mark.parametrize("centre, trap", [
    # An unchecked x - 1 from (0, 1, 0) aliases the last voxel of the row below.
    ((0, 1, 0), (cfg.NX - 1, 0, 0)),
    # An unchecked x + 1 from (NX-1, 0, 0) aliases the first voxel of the next row.
    ((cfg.NX - 1, 0, 0), (0, 1, 0)),
    # Index -1 would wrap to the last voxel of the map.
    ((0, 0, 0), (cfg.NX - 1, cfg.NY - 1, cfg.NZ - 1)),
    # One past the end would wrap to voxel 0.
    ((cfg.NX - 1, cfg.NY - 1, cfg.NZ - 1), (0, 0, 0)),
])
def test_4_boundary_voxel_does_not_alias(centre, trap):
    point = voxel_centre(*centre)
    # The trap plane passes through the point, so it would win if it were read.
    vmap = set_plane(VoxelMap.empty(), trap, point, (0, 0, 1))
    res = run_points([point], vmap)
    assert_empty(res, ZERO, processed=1, rejected=1)


def test_5_all_neighbours_invalid_is_rejected():
    # Descriptors are present but none is flagged valid.
    vmap = ground_map()
    vmap.valid[:] = 0
    res = run_points([[0.0, 0.0, 0.05]], vmap)
    assert_empty(res, ZERO, processed=1, rejected=1)


def test_7_two_constraints_accumulate():
    points = [[0.1, -0.05, 0.125], [-0.2, 0.15, -0.0625]]
    res = run_points(points, ground_map())
    H1, g1, c1 = expected_from([-0.05, -0.1, 0, 0, 0, 1], 0.125)
    H2, g2, c2 = expected_from([0.15, 0.2, 0, 0, 0, 1], -0.0625)
    assert res.inlier_count == 2
    assert np.array_equal(res.H, np.array([a + b for a, b in zip(H1, H2)], dtype=np.float64))
    assert np.array_equal(res.g, np.array([a + b for a, b in zip(g1, g2)], dtype=np.float64))
    assert res.cost == float(c1 + c2)
    # Spot-check against hand arithmetic: H(5,5) counts the points, H(0,0) = 0.05^2 + 0.15^2.
    H = unpack_h(res.H)
    assert H[5, 5] == 2.0
    assert H[0, 0] == pytest.approx(0.0025 + 0.0225, rel=1e-6)
    assert res.g[5] == pytest.approx(0.125 - 0.0625)


# ---- plane selection -------------------------------------------------------

def test_selects_minimum_absolute_residual():
    vmap = ground_map(CENTRE, z=0.20)
    set_plane(vmap, (17, 16, 4), (0.5, 0.0, -0.05), (0, 0, 1))
    res = run_points([[0.0, 0.0, 0.0]], vmap)
    assert res.g[5] == pytest.approx(0.05)   # the plane at z = -0.05 won, r = +0.05


def test_tie_keeps_first_candidate_in_scan_order():
    # |r| is 0.1 for both; (16,16,4) is visited before (17,16,4).
    vmap = ground_map(CENTRE, z=0.1)
    set_plane(vmap, (17, 16, 4), (0.5, 0.0, -0.1), (0, 0, 1))
    res = run_points([[0.0, 0.0, 0.0]], vmap)
    assert res.inlier_count == 1
    assert res.g[5] == float(F(0.0) - F(0.1))


def test_neighbour_radius():
    for voxel, found in [((17, 17, 5), True), ((15, 15, 3), True), ((18, 16, 4), False),
                         ((16, 16, 6), False)]:
        vmap = set_plane(VoxelMap.empty(), voxel, (0.0, 0.0, 0.0), (0, 0, 1))
        assert run_points([[0.0, 0.0, 0.0]], vmap).inlier_count == int(found)


def test_inlier_threshold_is_strict():
    below = np.nextafter(F(cfg.RESIDUAL_THRESHOLD), F(0))
    for z, inlier in [(F(cfg.RESIDUAL_THRESHOLD), False), (below, True), (0.6, False)]:
        res = run_points([[0.0, 0.0, 0.0]], ground_map(CENTRE, z=-float(z)))
        assert res.inlier_count == int(inlier)
        assert res.processed_count == 1 and res.rejected_count == int(not inlier)


def test_normal_sign_does_not_change_the_result():
    R_gt, t_gt = gen.ground_truth_pose()
    R, t = gen.predicted_pose(R_gt, t_gt)
    scan = gen.make_scan(R_gt, t_gt, spacing=0.4, n_clutter=10)
    vmap = build_voxel_map(gen.make_map_points(spacing=0.1))
    a = run(scan, len(scan), vmap, R, t)
    vmap.desc[:, 3:] *= -1
    b = run(scan, len(scan), vmap, R, t)
    assert a.inlier_count == b.inlier_count > 100
    assert np.array_equal(a.H, b.H) and np.array_equal(a.g, b.g) and a.cost == b.cost


# ---- edge cases ---------------------------------------------------------

def test_zero_points():
    assert_empty(run_points(np.zeros((0, 3)), ground_map()), ZERO, 0, 0)


def test_point_count_overflow_aborts():
    scan = np.zeros((4, 3), dtype=F)
    res = run(scan, cfg.MAX_POINTS + 1, ground_map(), *IDENTITY)
    assert_empty(res, ZERO | cfg.STATUS_POINT_OVERFLOW, 0, 0)


def test_max_points_is_accepted():
    scan = np.zeros((cfg.MAX_POINTS, 3), dtype=F)
    scan[:, 0] = 100.0   # all outside: cheap, but every point is visited
    res = run(scan, cfg.MAX_POINTS, ground_map(), *IDENTITY)
    assert_empty(res, ZERO | OOB, cfg.MAX_POINTS, cfg.MAX_POINTS)


def test_all_points_outside_map():
    res = run_points([[50, 0, 0], [0, -50, 0], [0, 0, 9]], ground_map())
    assert_empty(res, ZERO | OOB, 3, 3)


def test_empty_map():
    assert_empty(run_points([[0, 0, 0], [1, 1, 0]], VoxelMap.empty()), ZERO, 2, 2)


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_non_finite_points_are_rejected(bad):
    points = [[bad, 0, 0], [0, bad, 0], [0, 0, bad], [0.0, 0.0, 0.125]]
    res = run_points(points, ground_map())
    assert res.inlier_count == 1 and res.rejected_count == 3
    assert res.status == OK | OOB
    assert np.all(np.isfinite(res.H)) and np.all(np.isfinite(res.g))
    assert res.cost == 0.015625


@pytest.mark.parametrize("bad", [np.nan, np.inf])
@pytest.mark.parametrize("where", ["R", "t"])
def test_non_finite_pose_aborts(bad, where):
    R, t = np.eye(3), np.zeros(3)
    (R if where == "R" else t).flat[1] = bad
    res = run_points([[0.0, 0.0, 0.125]], ground_map(), pose=(R, t))
    assert_empty(res, ZERO | cfg.STATUS_BAD_POSE, 0, 0)


def test_zero_length_normal_is_treated_as_invalid():
    vmap = set_plane(VoxelMap.empty(), CENTRE, (0, 0, 0), (0, 0, 0))
    assert_empty(run_points([[0.0, 0.0, 0.05]], vmap), ZERO, 1, 1)
    # It must not shadow a real plane next to it either.
    set_plane(vmap, (17, 16, 4), (0.5, 0.0, 0.0), (0, 0, 1))
    res = run_points([[0.0, 0.0, 0.05]], vmap)
    assert res.inlier_count == 1 and res.g[5] == pytest.approx(0.05)


@pytest.mark.parametrize("bad", [np.nan, np.inf])
def test_non_finite_descriptor_never_wins(bad):
    vmap = set_plane(VoxelMap.empty(), CENTRE, (0, 0, bad), (0, 0, 1))
    assert_empty(run_points([[0.0, 0.0, 0.05]], vmap), ZERO, 1, 1)
    set_plane(vmap, (17, 16, 4), (0.5, 0.0, 0.0), (0, 0, 1))
    res = run_points([[0.0, 0.0, 0.05]], vmap)
    assert res.inlier_count == 1 and np.all(np.isfinite(res.H))


def test_extreme_pose_rejects_everything():
    res = run_points([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]], ground_map(),
                     pose=(np.eye(3), np.array([1e30, 0.0, 0.0])))
    assert_empty(res, ZERO | OOB, 2, 2)
    huge = run_points([[3e38, 3e38, 3e38]], ground_map(), pose=(np.eye(3) * 2, np.zeros(3)))
    assert_empty(huge, ZERO | OOB, 1, 1)


def test_single_plane_gives_singular_h():
    points = [[0.1 * i, 0.07 * j, 0.01] for i in range(-3, 4) for j in range(-3, 4)]
    res = run_points(points, ground_map())
    assert res.inlier_count == 49
    assert np.linalg.matrix_rank(unpack_h(res.H)) == 3   # z, roll, pitch only


def test_scan_shorter_than_num_points_is_a_host_error():
    with pytest.raises(ValueError):
        run(np.zeros((2, 3), dtype=F), 3, ground_map(), *IDENTITY)


# ---- whole-scan properties -------------------------------------------------

@pytest.fixture(scope="module")
def room():
    R_gt, t_gt = gen.ground_truth_pose()
    R, t = gen.predicted_pose(R_gt, t_gt)
    scan = gen.make_scan(R_gt, t_gt)
    vmap = build_voxel_map(gen.make_map_points())
    return scan, vmap, R, t


def test_room_counters_are_consistent(room):
    scan, vmap, R, t = room
    stats = {}
    res = run(scan, len(scan), vmap, R, t, stats=stats)
    assert res.processed_count == len(scan) == res.inlier_count + res.rejected_count
    assert res.rejected_count == stats["out_of_grid"] + stats["no_candidate"] + stats["over_threshold"]
    assert min(stats["out_of_grid"], stats["no_candidate"], stats["over_threshold"]) > 0
    assert res.status == OK | OOB
    assert res.inlier_count > 0.95 * len(scan)


def test_room_is_deterministic(room):
    scan, vmap, R, t = room
    a, b = run(scan, len(scan), vmap, R, t), run(scan, len(scan), vmap, R, t)
    assert np.array_equal(a.H, b.H) and np.array_equal(a.g, b.g) and a.cost == b.cost


def test_room_float32_close_to_float64(room):
    scan, vmap, R, t = room
    a = run(scan, len(scan), vmap, R, t, np.float32)
    b = run(scan, len(scan), vmap, R, t, np.float64)
    assert abs(a.inlier_count - b.inlier_count) <= 2
    assert np.max(np.abs(a.H - b.H)) < 1e-4 * np.max(np.abs(b.H))
    assert np.max(np.abs(a.g - b.g)) < 1e-3 * np.max(np.abs(b.g))


def test_num_points_selects_a_prefix(room):
    scan, vmap, R, t = room
    a = run(scan, 500, vmap, R, t)
    b = run(scan[:500], 500, vmap, R, t)
    assert a.processed_count == 500 and np.array_equal(a.H, b.H)


def test_result_file_round_trip(room, tmp_path):
    scan, vmap, R, t = room
    res = run(scan, 300, vmap, R, t)
    write_result(tmp_path / "r.txt", res)
    back = read_result(tmp_path / "r.txt")
    assert np.array_equal(back.H, res.H) and np.array_equal(back.g, res.g)
    assert back.cost == res.cost
    assert (back.inlier_count, back.processed_count, back.rejected_count, back.status) == (
        res.inlier_count, res.processed_count, res.rejected_count, res.status)
