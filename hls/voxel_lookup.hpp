#ifndef VOXLIO_VOXEL_LOOKUP_HPP
#define VOXLIO_VOXEL_LOOKUP_HPP

#include "voxlio_types.hpp"

// index = iz * NX * NY + iy * NX + ix. Only call with validated indices.
uint32_t flatten_index(int ix, int iy, int iz);

// Stages 4-7: sequential search of the (2R+1)^3 neighbourhood around
// (ix, iy, iz) for the usable voxel with the smallest |residual|. Ties keep
// the first candidate in dz, dy, dx order. Returns false when no candidate
// has a comparable residual.
bool find_best_plane(const Point3D &p, int ix, int iy, int iz,
                     const VoxelEntry voxel_map[NUM_VOXELS],
                     VoxelEntry &best_voxel, compute_t &best_r);

#endif // VOXLIO_VOXEL_LOOKUP_HPP
