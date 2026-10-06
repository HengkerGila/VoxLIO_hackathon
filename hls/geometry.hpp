#ifndef VOXLIO_GEOMETRY_HPP
#define VOXLIO_GEOMETRY_HPP

#include "voxlio_types.hpp"

// Stage 6: signed point-to-plane residual r = n . (p - c).
// Returns false when the descriptor is unusable (valid flag clear, or a
// zero-length normal); the residual must then be ignored.
bool evaluate_voxel(const Point3D &p, const VoxelEntry &v, compute_t &residual);

// Stage 9: J = [p x n, n] for the state ordering [wx, wy, wz, tx, ty, tz]
// and a left-multiplicative (map-frame) perturbation p' = Exp(w) p + t.
void compute_jacobian(const Point3D &p, const VoxelEntry &v, compute_t J[6]);

#endif // VOXLIO_GEOMETRY_HPP
