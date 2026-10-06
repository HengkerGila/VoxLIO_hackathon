#include "voxel_index.hpp"

// One axis. The range test is done on the scaled grid coordinate rather than
// on the metric one: (v - v_min) can round up to exactly the far edge in
// float, and testing the value that is actually truncated makes 0 <= idx < n
// hold by construction. NaN fails both comparisons and is rejected.
static bool axis_index(coord_t v, coord_t v_min, int n, int &idx) {
    const coord_t inv_voxel_size = 1.0f / VOXEL_SIZE;
    const grid_t f = (v - v_min) * inv_voxel_size;
    if (!(f >= 0 && f < n)) {
        idx = 0;
        return false;
    }
    idx = (int)f; // f >= 0, so truncation is floor
    return true;
}

bool voxel_index(const Point3D &p, int &ix, int &iy, int &iz) {
    const bool okx = axis_index(p.x, MAP_X_MIN, NX, ix);
    const bool oky = axis_index(p.y, MAP_Y_MIN, NY, iy);
    const bool okz = axis_index(p.z, MAP_Z_MIN, NZ, iz);
    return okx && oky && okz;
}
