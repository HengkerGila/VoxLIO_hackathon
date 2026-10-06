#include "geometry.hpp"

bool evaluate_voxel(const Point3D &p, const VoxelEntry &v, compute_t &residual) {
    residual = v.nx * (p.x - v.cx) + v.ny * (p.y - v.cy) + v.nz * (p.z - v.cz);
    const bool zero_normal = (v.nx == 0) && (v.ny == 0) && (v.nz == 0);
    return v.valid != 0 && !zero_normal;
}

void compute_jacobian(const Point3D &p, const VoxelEntry &v, compute_t J[6]) {
    J[0] = p.y * v.nz - p.z * v.ny;
    J[1] = p.z * v.nx - p.x * v.nz;
    J[2] = p.x * v.ny - p.y * v.nx;
    J[3] = v.nx;
    J[4] = v.ny;
    J[5] = v.nz;
}
