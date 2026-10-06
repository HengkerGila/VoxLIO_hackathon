import numpy as np
import pytest

from reference import config as cfg
from reference import geometry as geo

F = np.float32
MINS, INV = geo.grid_constants(F)


def f32(*values):
    return tuple(F(v) for v in values)


def index_of(x, y, z):
    return geo.voxel_index(f32(x, y, z), MINS, INV)


# ---- point transform -------------------------------------------------------

def test_transform_identity():
    R = f32(1, 0, 0, 0, 1, 0, 0, 0, 1)
    assert geo.transform_point(R, f32(0, 0, 0), f32(1.5, -2.25, 0.75)) == f32(1.5, -2.25, 0.75)


def test_transform_rotation_and_translation():
    # +90 degrees about z maps x to y, then translate.
    R = f32(0, -1, 0, 1, 0, 0, 0, 0, 1)
    p = geo.transform_point(R, f32(10, 20, 30), f32(1, 2, 3))
    assert p == f32(8, 21, 33)


def test_transform_matches_matrix_form():
    rng = np.random.RandomState(0)
    R = geo.rpy_to_matrix(0.1, -0.2, 0.3)
    t = rng.uniform(-1, 1, 3)
    q = rng.uniform(-5, 5, 3)
    p = geo.transform_point(R.reshape(9), t, q)
    np.testing.assert_allclose(p, R @ q + t, rtol=0, atol=1e-12)


# ---- voxel index, bounds check ---------------------------------------------

def test_voxel_index_origin_is_centre_voxel():
    assert index_of(0.0, 0.0, 0.0) == (16, 16, 4)


def test_voxel_index_negative_world_coordinates():
    assert index_of(-8.25, -8.25, -2.25) == (0, 0, 0)
    assert index_of(-8.0, -7.6, -1.9) == (0, 1, 0)


def test_voxel_index_rejects_outside():
    x_max = cfg.MAP_MIN[0] + cfg.NX * cfg.VOXEL_SIZE
    assert index_of(x_max, 0, 0) is None            # upper bound is exclusive
    assert index_of(np.nextafter(F(-8.25), F(-9)), 0, 0) is None
    assert index_of(0, 100.0, 0) is None
    assert index_of(0, 0, -2.5) is None


def test_voxel_index_last_voxel_below_upper_bound():
    x_max = F(cfg.MAP_MIN[0] + cfg.NX * cfg.VOXEL_SIZE)
    below = np.nextafter(x_max, F(0))
    result = index_of(below, 0, 0)
    # Either accepted into the last voxel or rejected, never index NX.
    assert result is None or result[0] == cfg.NX - 1


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_voxel_index_rejects_non_finite(bad):
    with np.errstate(all="ignore"):
        assert index_of(bad, 0, 0) is None
        assert index_of(0, bad, 0) is None
        assert index_of(0, 0, bad) is None


def test_voxel_index_always_in_range():
    rng = np.random.RandomState(1)
    points = rng.uniform(-9, 9, (20000, 3)).astype(F)
    # Pile points onto the faces of the grid, where rounding matters.
    edges = np.array(cfg.MAP_MIN) + np.array(cfg.GRID_DIMS) * cfg.VOXEL_SIZE
    points[:5000] = np.nextafter(F(edges), F(0)) + rng.randint(-2, 3, (5000, 3)) * F(1e-6)
    for p in points:
        result = geo.voxel_index(tuple(p), MINS, INV)
        if result is not None:
            assert all(0 <= i < n for i, n in zip(result, cfg.GRID_DIMS))


# ---- flattened addressing --------------------------------------------------

def test_flatten_index_formula_and_corners():
    assert geo.flatten_index(0, 0, 0) == 0
    assert geo.flatten_index(1, 0, 0) == 1
    assert geo.flatten_index(0, 1, 0) == cfg.NX
    assert geo.flatten_index(0, 0, 1) == cfg.NX * cfg.NY
    assert geo.flatten_index(cfg.NX - 1, cfg.NY - 1, cfg.NZ - 1) == cfg.NUM_VOXELS - 1


def test_flatten_index_is_a_bijection():
    seen = {geo.flatten_index(ix, iy, iz)
            for iz in range(cfg.NZ) for iy in range(cfg.NY) for ix in range(cfg.NX)}
    assert seen == set(range(cfg.NUM_VOXELS))


# ---- residual --------------------------------------------------------------

def test_residual_known_distance():
    # Plane z = 1 with normal +z; point 0.25 above it.
    assert geo.plane_residual(f32(3, 4, 1.25), f32(0, 0, 1), f32(0, 0, 1)) == F(0.25)


def test_residual_is_signed():
    assert geo.plane_residual(f32(0, 0, 0.5), f32(0, 0, 1), f32(0, 0, 1)) == F(-0.5)


def test_residual_zero_on_plane():
    n = np.array([1.0, 2.0, 2.0]) / 3.0
    c = np.array([0.5, -1.0, 2.0])
    on_plane = c + np.cross(n, [0.3, 0.1, -0.7])
    assert abs(geo.plane_residual(on_plane, c, n)) < 1e-15


# ---- Jacobian --------------------------------------------------------------

def test_jacobian_manual_case():
    # Hand-computed check case.
    assert geo.jacobian(f32(1, 2, 3), f32(0, 0, 1)) == f32(2, -1, 0, 0, 0, 1)


def test_jacobian_matches_finite_differences():
    """J must be the derivative of the residual under the left perturbation
    that apply_delta implements; this ties the convention together."""
    rng = np.random.RandomState(2)
    R = geo.rpy_to_matrix(0.2, -0.1, 0.4)
    t = np.array([0.3, -0.2, 0.1])
    q = rng.uniform(-3, 3, 3)
    n = rng.normal(size=3)
    n /= np.linalg.norm(n)
    c = rng.uniform(-3, 3, 3)

    def residual(delta):
        R2, t2 = geo.apply_delta(R, t, delta)
        return geo.plane_residual(R2 @ q + t2, c, n)

    J = geo.jacobian(R @ q + t, n)
    eps = 1e-6
    for k in range(6):
        delta = np.zeros(6)
        delta[k] = eps
        numeric = (residual(delta) - residual(-delta)) / (2 * eps)
        assert numeric == pytest.approx(J[k], abs=1e-8)


# ---- packing and host math -------------------------------------------------

def test_packed_triangle_order():
    # H00 / H01 H11 / H02 H12 H22 / ... (packed order)
    assert geo.TRI[:6] == [(0, 0), (0, 1), (1, 1), (0, 2), (1, 2), (2, 2)]
    assert len(geo.TRI) == 21 and geo.TRI[-1] == (5, 5)
    assert all(k == i + j * (j + 1) // 2 for k, (i, j) in enumerate(geo.TRI))


def test_unpack_h_is_symmetric():
    H = geo.unpack_h(np.arange(21.0))
    assert np.array_equal(H, H.T)
    assert H[0, 5] == 15 and H[5, 5] == 20 and H[1, 2] == 4


def test_apply_delta_composes_on_the_left():
    R = geo.rpy_to_matrix(0.1, 0.2, 0.3)
    t = np.array([1.0, 2.0, 3.0])
    delta = np.array([0.01, -0.02, 0.03, 0.1, 0.2, -0.1])
    R2, t2 = geo.apply_delta(R, t, delta)
    q = np.array([0.5, -1.5, 2.0])
    np.testing.assert_allclose(R2 @ q + t2, geo.so3_exp(delta[:3]) @ (R @ q + t) + delta[3:], atol=1e-14)
    np.testing.assert_allclose(R2 @ R2.T, np.eye(3), atol=1e-14)
