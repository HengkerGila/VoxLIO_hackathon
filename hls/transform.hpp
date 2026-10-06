#ifndef VOXLIO_TRANSFORM_HPP
#define VOXLIO_TRANSFORM_HPP

#include "voxlio_types.hpp"

// Stage 1: p_map = R * p + t.
Point3D transform_point(const Point3D &p, const Pose3D &pose);

#endif // VOXLIO_TRANSFORM_HPP
