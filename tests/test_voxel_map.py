import numpy as np

from reference import config as cfg
from reference import geometry as geo
from reference.voxel_map import build_voxel_map, orient_normal, voxel_indices

from maps import CENTRE, voxel_centre


def plane_points(centre, normal, n=200, half=0.2, noise=0.0, seed=0):
    """Points on the plane through `centre`, within +/-half along two in-plane axes."""
    rng = np.random.RandomState(seed)
    normal = np.asarray(normal, dtype=np.float64) / np.linalg.norm(normal)
    u = np.cross(normal, [1.0, 0.0, 0.0] if abs(normal[0]) < 0.9 else [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    ab = rng.uniform(-half, half, (n, 2))
    return (np.asarray(centre) + ab[:, :1] * u + ab[:, 1:] * v
            + rng.normal(0, noise, (n, 1)) * normal)


def test_vectorised_index_matches_scalar_index():
    rng = np.random.RandomState(3)
    points = rng.uniform(-9, 9, (5000, 3)).astype(np.float32)
    inside, flat = voxel_indices(points)
    mins, inv = geo.grid_constants(np.float32)
    for p, ok, k in zip(points, inside, flat):
        expected = geo.voxel_index(tuple(p), mins, inv)
        assert ok == (expected is not None)
        if ok:
            assert k == geo.flatten_index(*expected)


def test_recovers_horizontal_plane():
    centre = voxel_centre(*CENTRE)
    vmap = build_voxel_map(plane_points(centre, [0, 0, 1], noise=0.005))
    k = geo.flatten_index(*CENTRE)
    assert vmap.valid.sum() == 1 and vmap.valid[k] == 1
    np.testing.assert_allclose(vmap.desc[k, 3:], [0, 0, 1], atol=0.02)
    assert abs(vmap.desc[k, 2] - centre[2]) < 0.002


def test_recovers_tilted_plane_exactly_without_noise():
    centre = voxel_centre(*CENTRE)
    normal = np.array([1.0, 2.0, 2.0]) / 3.0
    vmap = build_voxel_map(plane_points(centre, normal, half=0.1))
    k = geo.flatten_index(*CENTRE)
    assert vmap.valid[k] == 1
    np.testing.assert_allclose(vmap.desc[k, 3:], normal, atol=1e-5)
    # The centroid lies on the plane and the stored normal has unit length.
    assert abs(np.dot(normal, vmap.desc[k, :3] - centre)) < 1e-6
    assert abs(np.linalg.norm(vmap.desc[k, 3:]) - 1.0) < 1e-6


def test_too_few_points_is_invalid():
    centre = voxel_centre(*CENTRE)
    vmap = build_voxel_map(plane_points(centre, [0, 0, 1], n=9), min_points=10)
    assert vmap.valid.sum() == 0
    assert vmap.count[geo.flatten_index(*CENTRE)] == 9


def test_points_outside_grid_are_ignored():
    far = np.array([[100.0, 0, 0]] * 50) + plane_points((0, 0, 0), [0, 0, 1], n=50)
    assert build_voxel_map(far).valid.sum() == 0


def test_normal_orientation_is_deterministic():
    assert np.array_equal(orient_normal(np.array([0.0, 0.0, -1.0])), [0, 0, 1])
    assert np.array_equal(orient_normal(np.array([-0.8, 0.6, 0.0])), [0.8, -0.6, 0.0])
    assert np.array_equal(orient_normal(np.array([0.1, 0.2, 0.9])), [0.1, 0.2, 0.9])

    centre = voxel_centre(*CENTRE)
    for normal in ([0, 0, 1], [0, 0, -1]):
        vmap = build_voxel_map(plane_points(centre, normal))
        assert vmap.desc[geo.flatten_index(*CENTRE), 5] > 0.99


def test_thickness_gate_rejects_two_planes_in_one_voxel():
    centre = np.array(voxel_centre(*CENTRE))
    corner = np.vstack([plane_points(centre, [0, 0, 1], half=0.2, seed=1),
                        plane_points(centre, [1, 0, 0], half=0.2, seed=2)])
    assert build_voxel_map(corner).valid.sum() == 1
    assert build_voxel_map(corner, max_thickness_ratio=0.05).valid.sum() == 0
    # A clean plane passes the same gate.
    flat = plane_points(centre, [0, 0, 1], noise=0.005)
    assert build_voxel_map(flat, max_thickness_ratio=0.05).valid.sum() == 1


def test_save_load_round_trip(tmp_path):
    vmap = build_voxel_map(plane_points(voxel_centre(*CENTRE), [0, 0, 1], noise=0.005))
    vmap.save(tmp_path)
    loaded = type(vmap).load(tmp_path)
    assert np.array_equal(loaded.desc, vmap.desc) and np.array_equal(loaded.valid, vmap.valid)
    assert loaded.desc.shape == (cfg.NUM_VOXELS, 6)
