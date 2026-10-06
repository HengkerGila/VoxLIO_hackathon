#ifndef VOXLIO_ACCUMULATOR_HPP
#define VOXLIO_ACCUMULATOR_HPP

#include "voxlio_types.hpp"

void clear_result(VoxLIOResult &result);

// Stage 11: H += J^T J, g += J^T r, cost += r^2, inlier_count += 1.
// Only call for accepted inliers.
void accumulate_constraint(const compute_t J[6], compute_t residual,
                           VoxLIOResult &result);

#endif // VOXLIO_ACCUMULATOR_HPP
