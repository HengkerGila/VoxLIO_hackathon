#include "voxel_lookup.hpp"

#include "geometry.hpp"

uint32_t flatten_index(int ix, int iy, int iz) {
    return (uint32_t)((iz * NY + iy) * NX + ix);
}

bool find_best_plane(const Point3D &p, int ix, int iy, int iz,
                     const VoxelEntry voxel_map[NUM_VOXELS],
                     VoxelEntry &best_voxel, compute_t &best_r) {
    bool found = false;
    compute_t best_abs = compute_max();

    best_r = 0;
    best_voxel.cx = 0;
    best_voxel.cy = 0;
    best_voxel.cz = 0;
    best_voxel.nx = 0;
    best_voxel.ny = 0;
    best_voxel.nz = 0;
    best_voxel.valid = 0;

NEIGHBOR_Z:
    for (int dz = -NEIGHBOR_RADIUS; dz <= NEIGHBOR_RADIUS; ++dz) {
    NEIGHBOR_Y:
        for (int dy = -NEIGHBOR_RADIUS; dy <= NEIGHBOR_RADIUS; ++dy) {
        NEIGHBOR_X:
            for (int dx = -NEIGHBOR_RADIUS; dx <= NEIGHBOR_RADIUS; ++dx) {
                const int cx = ix + dx;
                const int cy = iy + dy;
                const int cz = iz + dz;

                // Every address is range-checked per axis before it is formed.
                const bool in_grid = cx >= 0 && cx < NX && cy >= 0 && cy < NY &&
                                     cz >= 0 && cz < NZ;
                if (!in_grid) {
                    continue;
                }

                const VoxelEntry v = voxel_map[flatten_index(cx, cy, cz)];

                compute_t r;
                if (!evaluate_voxel(p, v, r)) {
                    continue;
                }

                const compute_t abs_r = vox_abs(r);
                if (abs_r < best_abs) {
                    best_abs = abs_r;
                    best_r = r;
                    best_voxel = v;
                    found = true;
                }
            }
        }
    }
    return found;
}
