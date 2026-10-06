#ifndef VOXLIO_TYPES_HPP
#define VOXLIO_TYPES_HPP

#include <stdint.h>

#include <limits>

#include "voxlio_config.hpp"

// Numeric categories.
//   coord_t   positions: scan points, centroids, pose translation
//   normal_t  unit-magnitude quantities: plane normals, rotation matrix
//   compute_t per-point intermediates: residual, Jacobian
//   accum_t   H, g and cost accumulators
// grid_t holds a position in voxel units on its way to becoming an index.
//
// Phase A (default) is float everywhere. Defining VOXLIO_FIXED_POINT switches
// every category to ap_fixed with the widths from voxlio_config.hpp.
#ifdef VOXLIO_FIXED_POINT

#include <ap_fixed.h>

// coord_t and compute_t saturate so that an out-of-range value stays out of
// range: a far-away point must not wrap around into the map and be accepted.
typedef ap_fixed<VOXLIO_COORD_W, VOXLIO_COORD_I, VOXLIO_QUANT, AP_SAT> coord_t;
typedef ap_fixed<VOXLIO_NORMAL_W, VOXLIO_NORMAL_I> normal_t;
typedef ap_fixed<VOXLIO_COMPUTE_W, VOXLIO_COMPUTE_I, VOXLIO_QUANT, AP_SAT> compute_t;
typedef ap_fixed<VOXLIO_ACCUM_W, VOXLIO_ACCUM_I> accum_t;

// Always truncates (toward minus infinity), so taking the integer part is an
// exact floor whatever VOXLIO_QUANT is; saturates like coord_t.
typedef ap_fixed<VOXLIO_COORD_W + 2, VOXLIO_COORD_I + 2, AP_TRN, AP_SAT> grid_t;

#else

typedef float coord_t;
typedef float normal_t;
typedef float compute_t;
typedef float accum_t;
typedef float grid_t;

#endif

struct Point3D {
    coord_t x;
    coord_t y;
    coord_t z;
};

// p_map = R * p_lidar + t. The host guarantees R is a valid rotation.
struct Pose3D {
    normal_t R[9]; // row-major 3 x 3
    coord_t t[3];
};

struct VoxelEntry {
    coord_t cx;
    coord_t cy;
    coord_t cz;

    normal_t nx;
    normal_t ny;
    normal_t nz;

    uint8_t valid;
};

struct VoxLIOResult {
    accum_t H[21]; // packed upper triangle, H[i + j*(j+1)/2] = H(i,j), i <= j
    accum_t g[6];
    accum_t cost;
    uint32_t inlier_count;
    uint32_t processed_count;
    uint32_t rejected_count;
    uint32_t status;
};

// ---- Small helpers that differ between float and fixed point ---------------

template <typename T>
inline T vox_abs(const T &x) {
    const T neg = -x;
    return (x < 0) ? neg : x;
}

#ifdef VOXLIO_FIXED_POINT

// Largest representable compute_t: the "infinity" that starts the minimum search.
inline compute_t compute_max() {
    compute_t m = 0;
    m.range(VOXLIO_COMPUTE_W - 2, 0) = -1;
    return m;
}

// Fixed-point values are always finite.
inline bool vox_is_finite(const coord_t &) { return true; }
inline bool vox_is_finite(const normal_t &) { return true; }

#else

inline compute_t compute_max() { return std::numeric_limits<float>::infinity(); }

// False for NaN and +/-Inf; two comparators, no arithmetic.
inline bool vox_is_finite(float x) {
    return x >= -std::numeric_limits<float>::max() &&
           x <= std::numeric_limits<float>::max();
}

#endif

#endif // VOXLIO_TYPES_HPP
