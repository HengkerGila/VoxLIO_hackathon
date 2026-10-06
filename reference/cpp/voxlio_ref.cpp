#include "voxlio_ref.hpp"

#include <cmath>
#include <limits>

#include "voxlio_config.hpp"

RefResult voxlio_ref(const float *scan_xyz, uint32_t num_points,
                     const float *voxel_desc, const uint8_t *voxel_valid,
                     const float R[9], const float t[3]) {
    RefResult res = RefResult();

    bool pose_finite = true;
    for (int i = 0; i < 9; ++i) {
        pose_finite = pose_finite && std::isfinite(R[i]);
    }
    for (int i = 0; i < 3; ++i) {
        pose_finite = pose_finite && std::isfinite(t[i]);
    }

    uint32_t status = 0;
    if (num_points > MAX_POINTS) {
        status |= VOXLIO_STATUS_POINT_OVERFLOW;
    }
    if (!pose_finite) {
        status |= VOXLIO_STATUS_BAD_POSE;
    }
    const bool aborted = status != 0;

    const float map_min[3] = {MAP_X_MIN, MAP_Y_MIN, MAP_Z_MIN};
    const int dims[3] = {NX, NY, NZ};
    const float inv_voxel_size = 1.0f / VOXEL_SIZE;
    bool out_of_grid = false;

    for (uint32_t n = 0; !aborted && n < num_points; ++n) {
        res.processed_count++;

        const float x = scan_xyz[3 * n + 0];
        const float y = scan_xyz[3 * n + 1];
        const float z = scan_xyz[3 * n + 2];
        const float p[3] = {
            R[0] * x + R[1] * y + R[2] * z + t[0],
            R[3] * x + R[4] * y + R[5] * z + t[1],
            R[6] * x + R[7] * y + R[8] * z + t[2],
        };

        // Bounds check on the scaled coordinate; NaN/Inf fail it.
        int centre[3] = {0, 0, 0};
        bool inside = true;
        for (int a = 0; a < 3; ++a) {
            const float f = (p[a] - map_min[a]) * inv_voxel_size;
            if (!(f >= 0.0f && f < (float)dims[a])) {
                inside = false;
                break;
            }
            centre[a] = (int)std::floor(f);
        }
        if (!inside) {
            res.rejected_count++;
            out_of_grid = true;
            continue;
        }

        float best_abs = std::numeric_limits<float>::infinity();
        float best_r = 0.0f;
        const float *best = 0;

        for (int dz = -NEIGHBOR_RADIUS; dz <= NEIGHBOR_RADIUS; ++dz) {
            for (int dy = -NEIGHBOR_RADIUS; dy <= NEIGHBOR_RADIUS; ++dy) {
                for (int dx = -NEIGHBOR_RADIUS; dx <= NEIGHBOR_RADIUS; ++dx) {
                    const int cx = centre[0] + dx;
                    const int cy = centre[1] + dy;
                    const int cz = centre[2] + dz;
                    if (cx < 0 || cx >= NX || cy < 0 || cy >= NY || cz < 0 || cz >= NZ) {
                        continue;
                    }

                    const int index = cz * NX * NY + cy * NX + cx;
                    const float *v = voxel_desc + 6 * index;
                    if (!voxel_valid[index]) {
                        continue;
                    }
                    if (v[3] == 0.0f && v[4] == 0.0f && v[5] == 0.0f) {
                        continue; // zero-length normal: unusable descriptor
                    }

                    const float r = v[3] * (p[0] - v[0]) + v[4] * (p[1] - v[1]) +
                                    v[5] * (p[2] - v[2]);
                    if (std::fabs(r) < best_abs) {
                        best_abs = std::fabs(r);
                        best_r = r;
                        best = v;
                    }
                }
            }
        }

        if (!best) {
            res.rejected_count++;
            continue;
        }
        if (!(std::fabs(best_r) < RESIDUAL_THRESHOLD)) {
            res.rejected_count++;
            continue;
        }

        const float nx = best[3], ny = best[4], nz = best[5];
        const float J[6] = {
            p[1] * nz - p[2] * ny,
            p[2] * nx - p[0] * nz,
            p[0] * ny - p[1] * nx,
            nx,
            ny,
            nz,
        };

        int k = 0;
        for (int j = 0; j < 6; ++j) {
            for (int i = 0; i <= j; ++i) {
                res.H[k++] += J[i] * J[j];
            }
        }
        for (int i = 0; i < 6; ++i) {
            res.g[i] += J[i] * best_r;
        }
        res.cost += best_r * best_r;
        res.inlier_count++;
    }

    if (out_of_grid) {
        status |= VOXLIO_STATUS_OOB_PREVENTED;
    }
    if (res.inlier_count == 0) {
        status |= VOXLIO_STATUS_ZERO_INLIERS;
    }
    if (!aborted && res.inlier_count > 0) {
        status |= VOXLIO_STATUS_SUCCESS;
    }
    res.status = status;
    return res;
}
