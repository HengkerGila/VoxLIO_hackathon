#include "voxlio_core.hpp"

#include "accumulator.hpp"
#include "geometry.hpp"
#include "transform.hpp"
#include "voxel_index.hpp"
#include "voxel_lookup.hpp"

static bool pose_is_finite(const Pose3D &pose) {
    bool ok = true;
    for (int i = 0; i < 9; ++i) {
        ok = ok && vox_is_finite(pose.R[i]);
    }
    for (int i = 0; i < 3; ++i) {
        ok = ok && vox_is_finite(pose.t[i]);
    }
    return ok;
}

void voxlio_core(const Point3D scan[MAX_POINTS], uint32_t num_points,
                 const VoxelEntry voxel_map[NUM_VOXELS], const Pose3D pose,
                 VoxLIOResult &result) {
#pragma HLS INTERFACE m_axi port = scan offset = slave bundle = scan_bus
#pragma HLS INTERFACE m_axi port = voxel_map offset = slave bundle = map_bus
#pragma HLS INTERFACE s_axilite port = scan bundle = control
#pragma HLS INTERFACE s_axilite port = voxel_map bundle = control
#pragma HLS INTERFACE s_axilite port = num_points bundle = control
#pragma HLS INTERFACE s_axilite port = pose bundle = control
#pragma HLS INTERFACE s_axilite port = result bundle = control
#pragma HLS INTERFACE s_axilite port = return bundle = control

    const uint32_t c_max_points = MAX_POINTS; // pragmas do not expand macros
    const compute_t threshold = RESIDUAL_THRESHOLD;

    // Accumulate locally; the interface registers are written once at the end.
    VoxLIOResult acc;
    clear_result(acc);

    uint32_t status = 0;
    if (num_points > c_max_points) {
        status |= VOXLIO_STATUS_POINT_OVERFLOW;
    }
    if (!pose_is_finite(pose)) {
        status |= VOXLIO_STATUS_BAD_POSE;
    }

    bool out_of_grid = false;

    if (status == 0) {
    POINT_LOOP:
        for (uint32_t i = 0; i < num_points; ++i) {
#pragma HLS LOOP_TRIPCOUNT min = 0 max = c_max_points
            acc.processed_count += 1;

            const Point3D p = transform_point(scan[i], pose);

            int ix, iy, iz;
            if (!voxel_index(p, ix, iy, iz)) {
                acc.rejected_count += 1;
                out_of_grid = true;
                continue;
            }

            VoxelEntry best_voxel;
            compute_t best_r;
            if (!find_best_plane(p, ix, iy, iz, voxel_map, best_voxel, best_r)) {
                acc.rejected_count += 1;
                continue;
            }

            if (!(vox_abs(best_r) < threshold)) {
                acc.rejected_count += 1;
                continue;
            }

            compute_t J[6];
            compute_jacobian(p, best_voxel, J);
            accumulate_constraint(J, best_r, acc);
        }
    }

    if (out_of_grid) {
        status |= VOXLIO_STATUS_OOB_PREVENTED;
    }
    if (acc.inlier_count == 0) {
        status |= VOXLIO_STATUS_ZERO_INLIERS;
    }
    const uint32_t error_bits = VOXLIO_STATUS_BAD_POSE | VOXLIO_STATUS_POINT_OVERFLOW;
    if ((status & error_bits) == 0 && acc.inlier_count > 0) {
        status |= VOXLIO_STATUS_SUCCESS;
    }
    acc.status = status;

    result = acc;
}
