#ifndef VOXLIO_REF_HPP
#define VOXLIO_REF_HPP

#include <stdint.h>

// Plain C++ float reference of the VoxLIO front-end (implementation phase 4).
// A direct translation of the top-level pseudocode, kept free of HLS types
// and written independently of the hls/ modules so that comparing the two is
// a real check. It stays float when the HLS core is built in fixed point and
// is the baseline the fixed-point error is measured against.

struct RefResult {
    float H[21];
    float g[6];
    float cost;
    uint32_t inlier_count;
    uint32_t processed_count;
    uint32_t rejected_count;
    uint32_t status;
};

// scan_xyz     num_points x 3, LiDAR frame
// voxel_desc   NUM_VOXELS x 6: cx cy cz nx ny nz
// voxel_valid  NUM_VOXELS
// R, t         predicted pose, R row-major, p_map = R p + t
RefResult voxlio_ref(const float *scan_xyz, uint32_t num_points,
                     const float *voxel_desc, const uint8_t *voxel_valid,
                     const float R[9], const float t[3]);

#endif // VOXLIO_REF_HPP
