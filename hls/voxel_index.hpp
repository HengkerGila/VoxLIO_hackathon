#ifndef VOXLIO_VOXEL_INDEX_HPP
#define VOXLIO_VOXEL_INDEX_HPP

#include "voxlio_types.hpp"

// Stages 2 + 3: bounds check and voxel coordinate of a map-frame point.
// Returns false when the point is outside the grid (or is NaN/Inf); the
// indices are then 0 and must not be used.
bool voxel_index(const Point3D &p, int &ix, int &iy, int &iz);

#endif // VOXLIO_VOXEL_INDEX_HPP
