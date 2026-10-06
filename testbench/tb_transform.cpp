// Unit test: stage 1, point transform.

#include "tb_util.hpp"
#include "transform.hpp"

int main() {
    // Identity pose leaves the point unchanged.
    Point3D p = transform_point(tb::point(1.5f, -2.25f, 0.75f),
                                tb::pose(tb::IDENTITY_R, tb::ZERO_T));
    CHECK_NEAR(p.x, 1.5, tb::EXACT);
    CHECK_NEAR(p.y, -2.25, tb::EXACT);
    CHECK_NEAR(p.z, 0.75, tb::EXACT);

    // +90 degrees about z maps x to y; then translate.
    const float Rz[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    const float t[3] = {10, 20, 30};
    p = transform_point(tb::point(1, 2, 3), tb::pose(Rz, t));
    CHECK_NEAR(p.x, 8.0, tb::EXACT);
    CHECK_NEAR(p.y, 21.0, tb::EXACT);
    CHECK_NEAR(p.z, 33.0, tb::EXACT);

    // Pure translation.
    const float t2[3] = {-0.5f, 0.25f, 4.0f};
    p = transform_point(tb::point(0, 0, 0), tb::pose(tb::IDENTITY_R, t2));
    CHECK_NEAR(p.x, -0.5, tb::EXACT);
    CHECK_NEAR(p.y, 0.25, tb::EXACT);
    CHECK_NEAR(p.z, 4.0, tb::EXACT);

    // General rotation Rz(0.3) Ry(-0.2) Rx(0.1) against double arithmetic.
    const double cr = std::cos(0.1), sr = std::sin(0.1);
    const double cp = std::cos(-0.2), sp = std::sin(-0.2);
    const double cy = std::cos(0.3), sy = std::sin(0.3);
    const double Rd[9] = {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                          sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                          -sp,     cp * sr,                cp * cr};
    const double td[3] = {0.2, -0.1, 0.05};
    const double q[3] = {3.25, -4.5, 1.125};
    float Rf[9], tf[3];
    for (int i = 0; i < 9; ++i) {
        Rf[i] = (float)Rd[i];
    }
    for (int i = 0; i < 3; ++i) {
        tf[i] = (float)td[i];
    }
    p = transform_point(tb::point((float)q[0], (float)q[1], (float)q[2]), tb::pose(Rf, tf));
    const double tol = tb::FIXED_POINT ? 2e-3 : 2e-6;
    CHECK_NEAR(p.x, Rd[0] * q[0] + Rd[1] * q[1] + Rd[2] * q[2] + td[0], tol);
    CHECK_NEAR(p.y, Rd[3] * q[0] + Rd[4] * q[1] + Rd[5] * q[2] + td[1], tol);
    CHECK_NEAR(p.z, Rd[6] * q[0] + Rd[7] * q[1] + Rd[8] * q[2] + td[2], tol);

    return tb::finish("tb_transform");
}
