// Unit test: stage 11, H / g / cost accumulation (acceptance test 7).

#include "accumulator.hpp"
#include "tb_util.hpp"

int main() {
    VoxLIOResult res;
    // Start from garbage to prove clear_result resets every field.
    for (int k = 0; k < 21; ++k) {
        res.H[k] = 7;
    }
    for (int i = 0; i < 6; ++i) {
        res.g[i] = -3;
    }
    res.cost = 5;
    res.inlier_count = res.processed_count = res.rejected_count = res.status = 99;

    clear_result(res);
    bool all_zero = res.cost == 0 && res.inlier_count == 0 && res.processed_count == 0 &&
                    res.rejected_count == 0 && res.status == 0;
    for (int k = 0; k < 21; ++k) {
        all_zero = all_zero && res.H[k] == 0;
    }
    for (int i = 0; i < 6; ++i) {
        all_zero = all_zero && res.g[i] == 0;
    }
    CHECK(all_zero);

    // Two constraints with values that are exact in binary.
    const double J1[6] = {1, 2, 3, 4, 5, 6};
    const double J2[6] = {-1, 0.5, 0, 2, -2, 1};
    const double r1 = 0.5, r2 = -0.25;
    compute_t Ja[6], Jb[6];
    for (int i = 0; i < 6; ++i) {
        Ja[i] = J1[i];
        Jb[i] = J2[i];
    }

    accumulate_constraint(Ja, compute_t(r1), res);
    CHECK(res.inlier_count == 1);
    CHECK_NEAR(res.H[0], 1.0, tb::EXACT);
    CHECK_NEAR(res.cost, 0.25, tb::EXACT);

    accumulate_constraint(Jb, compute_t(r2), res);
    CHECK(res.inlier_count == 2);

    // Hand-computed entries.
    CHECK_NEAR(res.H[0], 1 + 1, tb::EXACT);        // H00
    CHECK_NEAR(res.H[1], 2 - 0.5, tb::EXACT);      // H01
    CHECK_NEAR(res.H[2], 4 + 0.25, tb::EXACT);     // H11
    CHECK_NEAR(res.H[3], 3 + 0, tb::EXACT);        // H02
    CHECK_NEAR(res.H[15], 6 - 1, tb::EXACT);       // H05
    CHECK_NEAR(res.H[19], 30 - 2, tb::EXACT);      // H45
    CHECK_NEAR(res.H[20], 36 + 1, tb::EXACT);      // H55
    CHECK_NEAR(res.g[0], 0.5 + 0.25, tb::EXACT);
    CHECK_NEAR(res.g[4], 2.5 + 0.5, tb::EXACT);
    CHECK_NEAR(res.g[5], 3 - 0.25, tb::EXACT);
    CHECK_NEAR(res.cost, 0.25 + 0.0625, tb::EXACT);

    // Every packed entry: H[i + j(j+1)/2] = sum J_i J_j for i <= j.
    for (int j = 0; j < 6; ++j) {
        for (int i = 0; i <= j; ++i) {
            CHECK_NEAR(res.H[i + j * (j + 1) / 2], J1[i] * J1[j] + J2[i] * J2[j], tb::EXACT);
        }
    }
    for (int i = 0; i < 6; ++i) {
        CHECK_NEAR(res.g[i], J1[i] * r1 + J2[i] * r2, tb::EXACT);
    }

    // The accumulator does not touch the other counters.
    CHECK(res.processed_count == 0 && res.rejected_count == 0 && res.status == 0);

    return tb::finish("tb_accumulator");
}
