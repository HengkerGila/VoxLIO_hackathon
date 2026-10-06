// Unit test: stage 6 (residual) and stage 9 (Jacobian).

#include "geometry.hpp"
#include "tb_util.hpp"

int main() {
    compute_t r = 0;

    // Plane z = 1 with normal +z.
    const VoxelEntry ground = tb::voxel(0, 0, 1, 0, 0, 1);
    CHECK(evaluate_voxel(tb::point(3, 4, 1.25f), ground, r));
    CHECK_NEAR(r, 0.25, tb::EXACT);

    // The residual is signed.
    CHECK(evaluate_voxel(tb::point(3, 4, 0.5f), ground, r));
    CHECK_NEAR(r, -0.5, tb::EXACT);

    // A point exactly on the plane has zero residual (acceptance test 2).
    CHECK(evaluate_voxel(tb::point(-2, 7, 1), ground, r));
    CHECK_NEAR(r, 0.0, tb::EXACT);

    // Oblique plane through (0.5, -1, 2) with normal (1, 2, 2) / 3.
    const VoxelEntry oblique = tb::voxel(0.5f, -1, 2, 1.0f / 3, 2.0f / 3, 2.0f / 3);
    const double tol = tb::FIXED_POINT ? 2e-3 : 1e-6;
    CHECK(evaluate_voxel(tb::point(1.5f, 0, 3), oblique, r));
    CHECK_NEAR(r, (1.0 * 1 + 2.0 * 1 + 2.0 * 1) / 3.0, tol);

    // Unusable descriptors: valid flag clear, or zero-length normal.
    CHECK(!evaluate_voxel(tb::point(0, 0, 1), tb::voxel(0, 0, 1, 0, 0, 1, 0), r));
    CHECK(!evaluate_voxel(tb::point(0, 0, 1), tb::voxel(0, 0, 1, 0, 0, 0, 1), r));

    // Acceptance test 6: p = [1,2,3], n = [0,0,1].
    compute_t J[6];
    compute_jacobian(tb::point(1, 2, 3), tb::voxel(0, 0, 0, 0, 0, 1), J);
    const double manual[6] = {2, -1, 0, 0, 0, 1};
    for (int i = 0; i < 6; ++i) {
        CHECK_NEAR(J[i], manual[i], tb::EXACT);
    }

    // The centroid does not enter the Jacobian.
    compute_jacobian(tb::point(1, 2, 3), tb::voxel(9, -9, 9, 0, 0, 1), J);
    for (int i = 0; i < 6; ++i) {
        CHECK_NEAR(J[i], manual[i], tb::EXACT);
    }

    // General case against double arithmetic: J = [p x n, n].
    const double p[3] = {1.5, -2.25, 0.75};
    const double n[3] = {2.0 / 7, -3.0 / 7, 6.0 / 7};
    compute_jacobian(tb::point(1.5f, -2.25f, 0.75f),
                     tb::voxel(0, 0, 0, (float)n[0], (float)n[1], (float)n[2]), J);
    CHECK_NEAR(J[0], p[1] * n[2] - p[2] * n[1], tol);
    CHECK_NEAR(J[1], p[2] * n[0] - p[0] * n[2], tol);
    CHECK_NEAR(J[2], p[0] * n[1] - p[1] * n[0], tol);
    CHECK_NEAR(J[3], n[0], tol);
    CHECK_NEAR(J[4], n[1], tol);
    CHECK_NEAR(J[5], n[2], tol);

    return tb::finish("tb_geometry");
}
