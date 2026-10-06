#include "accumulator.hpp"

// Row / column of each packed upper-triangle entry: k = row + col*(col+1)/2.
static const uint8_t TRI_ROW[21] = {0, 0, 1, 0, 1, 2, 0, 1, 2, 3, 0,
                                    1, 2, 3, 4, 0, 1, 2, 3, 4, 5};
static const uint8_t TRI_COL[21] = {0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4,
                                    4, 4, 4, 4, 5, 5, 5, 5, 5, 5};

void clear_result(VoxLIOResult &result) {
    for (int k = 0; k < 21; ++k) {
        result.H[k] = 0;
    }
    for (int i = 0; i < 6; ++i) {
        result.g[i] = 0;
    }
    result.cost = 0;
    result.inlier_count = 0;
    result.processed_count = 0;
    result.rejected_count = 0;
    result.status = 0;
}

void accumulate_constraint(const compute_t J[6], compute_t residual,
                           VoxLIOResult &result) {
    for (int k = 0; k < 21; ++k) {
        result.H[k] += J[TRI_ROW[k]] * J[TRI_COL[k]];
    }
    for (int i = 0; i < 6; ++i) {
        result.g[i] += J[i] * residual;
    }
    result.cost += residual * residual;
    result.inlier_count += 1;
}
