#ifndef VOXLIO_CONFIG_HPP
#define VOXLIO_CONFIG_HPP

// Single source of truth for every tunable constant.
//
// reference/config.py parses the plain numeric #defines in this file, so the
// Python golden model, the C++ reference and the HLS core cannot disagree.
// Keep every tunable as one literal on one line.

// ---- Scan buffer -----------------------------------------------------------
#define MAX_POINTS 32768

// ---- Voxel grid ------------------------------------------------------------
#define NX 32
#define NY 32
#define NZ 8

#define NUM_VOXELS (NX * NY * NZ)

#define VOXEL_SIZE 0.5f

// Minimum corner of the grid. The original example uses -8 / -8 / -2. The origin
// is shifted by half a voxel (0.25 m) so that voxel *centres*, not voxel
// faces, sit on the 0.5 m lattice: the synthetic ground plane (z = 0) and
// walls (x = 4, y = 4) then lie in the middle of a voxel instead of being
// split across two. Extent (16 x 16 x 4 m) and resolution are unchanged.
#define MAP_X_MIN -8.25f
#define MAP_Y_MIN -8.25f
#define MAP_Z_MIN -2.25f

// ---- Matching --------------------------------------------------------------
#define RESIDUAL_THRESHOLD 0.30f

#define NEIGHBOR_RADIUS 1

// ---- Fixed-point formats ---------------------------------------------------
// Used only when VOXLIO_FIXED_POINT is defined. <W, I> = total bits, integer
// bits including sign. The widths are the baseline starting values.
// With rounding they meet the selected tolerance on every synthetic case;
// with truncation they do not (results/verification/fixed_point_sweep.md).
// They have not been weighed against synthesis resource data yet.
// Override with -D to explore.
#ifndef VOXLIO_COORD_W
#define VOXLIO_COORD_W 24
#endif
#ifndef VOXLIO_COORD_I
#define VOXLIO_COORD_I 10
#endif
#ifndef VOXLIO_NORMAL_W
#define VOXLIO_NORMAL_W 18
#endif
#ifndef VOXLIO_NORMAL_I
#define VOXLIO_NORMAL_I 2
#endif
#ifndef VOXLIO_COMPUTE_W
#define VOXLIO_COMPUTE_W 32
#endif
#ifndef VOXLIO_COMPUTE_I
#define VOXLIO_COMPUTE_I 16
#endif
#ifndef VOXLIO_ACCUM_W
#define VOXLIO_ACCUM_W 64
#endif
#ifndef VOXLIO_ACCUM_I
#define VOXLIO_ACCUM_I 32
#endif
// Quantisation of coord_t and compute_t results: AP_TRN (truncate, biased by
// half an LSB) or AP_RND (round to nearest).
#ifndef VOXLIO_QUANT
#define VOXLIO_QUANT AP_RND
#endif

// ---- Status word (interface contract; not tunable) --------
// SUCCESS        run completed, no error bit, at least one inlier
// ZERO_INLIERS   inlier_count == 0 (always mirrors the counter)
// BAD_MAP_CONFIG reserved: the v0.1 grid is compile-time, nothing to reject
// BAD_POSE       R or t contains NaN/Inf; run aborted, no point processed
// POINT_OVERFLOW num_points > MAX_POINTS; run aborted, no point processed
// OOB_PREVENTED  at least one point fell outside the grid (or was NaN/Inf)
//                and was rejected before any memory access; informational
#define VOXLIO_STATUS_SUCCESS 0x01
#define VOXLIO_STATUS_ZERO_INLIERS 0x02
#define VOXLIO_STATUS_BAD_MAP_CONFIG 0x04
#define VOXLIO_STATUS_BAD_POSE 0x08
#define VOXLIO_STATUS_POINT_OVERFLOW 0x10
#define VOXLIO_STATUS_OOB_PREVENTED 0x20

#endif // VOXLIO_CONFIG_HPP
