#ifndef VOXLIO_CORE_HPP
#define VOXLIO_CORE_HPP

#include "voxlio_types.hpp"

// Top-level accelerator function. Reads num_points points
// from scan, matches each against the read-only voxel map under the predicted
// pose, and returns only the reduced normal equations and counters.
void voxlio_core(const Point3D scan[MAX_POINTS], uint32_t num_points,
                 const VoxelEntry voxel_map[NUM_VOXELS], const Pose3D pose,
                 VoxLIOResult &result);

#endif // VOXLIO_CORE_HPP
