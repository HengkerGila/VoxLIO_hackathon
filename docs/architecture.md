# VoxLIO v0.1 as built

The computation of each stage is derived in [mathematics.md](mathematics.md).
This document records the choices the code makes where the original design
left them open, and where it departs from the original examples.

## Pipeline to source

| Stage | Function | File |
|---|---|---|
| 1 Point transform | `transform_point` | `hls/transform.cpp` |
| 2 Bounds check, 3 Voxel index | `voxel_index` | `hls/voxel_index.cpp` |
| 4 Neighbour enumeration, 5 Voxel read, 7 Best plane | `find_best_plane`, `flatten_index` | `hls/voxel_lookup.cpp` |
| 6 Plane candidate evaluation | `evaluate_voxel` | `hls/geometry.cpp` |
| 9 Jacobian | `compute_jacobian` | `hls/geometry.cpp` |
| 8 Residual, 10 Inlier test, control, status | `voxlio_core` | `hls/voxlio_core.cpp` |
| 11 Accumulation | `accumulate_constraint` | `hls/accumulator.cpp` |

The same algorithm exists three times: `reference/voxlio_reference.py`
(golden model), `reference/cpp/voxlio_ref.cpp` (plain C++, one function, no
HLS types) and the modules above. The second and third were written
separately so that comparing them means something.

## Conventions

- Pose: `p_map = R * p_lidar + t`, `R` row-major.
- State ordering: `delta = [wx, wy, wz, tx, ty, tz]`.
- Perturbation: left-multiplicative, in the map frame:
  `p_map' = Exp(w) * p_map + tau`. Hence `J = [p_map x n, n]`.
- Host update: `R <- Exp(w) * R`, `t <- Exp(w) * t + tau`, with `delta` from
  `H * delta = -g`. Implemented in `reference/geometry.py`; a finite-difference
  test ties `J` to this update.
- `H` packing: upper triangle by columns, `H[i + j*(j+1)/2] = H(i,j)` for
  `i <= j`, column by column.
- Candidate order: `dz`, then `dy`, then `dx`, each from -1 to +1. A candidate
  replaces the best one only if its `|r|` is strictly smaller, so ties keep
  the first candidate visited.
- Voxel index: `floor((p - min) * (1 / VOXEL_SIZE))`, a multiplication by the
  reciprocal as section 11.3 asks.

## Interface

```cpp
void voxlio_core(const Point3D scan[MAX_POINTS], uint32_t num_points,
                 const VoxelEntry voxel_map[NUM_VOXELS], const Pose3D pose,
                 VoxLIOResult &result);
```

`scan` and `voxel_map` are AXI master ports on separate bundles (`scan_bus`,
`map_bus`); everything else is on one AXI-Lite bundle (`control`). The map is
read-only. Results are accumulated in local registers and written to the
interface once, at the end.

The host must supply a valid rotation matrix, a scan buffer holding at least
`num_points` points, and a map in which `valid` is set only for descriptors
with a unit normal.

## Status word and edge cases

| Bit | Name | Meaning |
|---|---|---|
| 0 | `SUCCESS` | The run completed, no error bit is set and there is at least one inlier. |
| 1 | `ZERO_INLIERS` | `inlier_count == 0`. Always mirrors the counter. |
| 2 | `BAD_MAP_CONFIG` | Reserved. The grid is fixed at compile time, so there is nothing to reject at run time. |
| 3 | `BAD_POSE` | `R` or `t` contains NaN or Inf. The run is aborted. |
| 4 | `POINT_OVERFLOW` | `num_points > MAX_POINTS`. The run is aborted. |
| 5 | `OOB_PREVENTED` | At least one point fell outside the grid and was rejected before any memory access. Informational; does not clear `SUCCESS`. |

An aborted run reads no point and returns zeros in every field except
`status`. Counters always satisfy
`processed_count == inlier_count + rejected_count`.

| Case | Behaviour |
|---|---|
| `num_points = 0` | Zero result, status `ZERO_INLIERS`. |
| `num_points > MAX_POINTS` | Aborted, status `POINT_OVERFLOW | ZERO_INLIERS`. |
| All points outside the map | All rejected, status `ZERO_INLIERS | OOB_PREVENTED`. |
| No valid neighbouring voxel | Point rejected. |
| Zero-length normal | Descriptor treated as invalid, whatever its `valid` flag says. Otherwise its residual would always be 0 and it would win every search. |
| NaN / Inf point | Fails the bounds check and is rejected; counted under `OOB_PREVENTED`. |
| NaN / Inf descriptor | Its residual is NaN or Inf, which never compares smaller, so it is never selected. |
| NaN / Inf pose | Aborted, status `BAD_POSE | ZERO_INLIERS`. |
| Empty map | All points rejected. |
| Singular `H` | Host responsibility: `solve_update` rejects it by condition number. |
| Extreme predicted pose | Points land outside the grid and are rejected. In fixed point, coordinates saturate instead of wrapping, so a far point cannot alias into the map. |
| Boundary voxel | Each neighbour coordinate is range-checked per axis before the address is formed; out-of-grid neighbours are skipped. |
| Negative world coordinates | Handled by subtracting the grid minimum. |

## Numeric modes

**Float (default).** Every category is `float`. Operations are written in one
fixed order and built with `-ffp-contract=off`, so the Python float32 model
and both C++ versions agree bit for bit.

**Fixed point (`-DVOXLIO_FIXED_POINT`).** All four categories switch to
`ap_fixed` at once; widths and the rounding mode come from
`voxlio_config.hpp` and can be overridden with `-D`.

| Type | Default | Holds | Overflow |
|---|---|---|---|
| `coord_t` | `<24,10>` | points, centroids, translation | saturates |
| `normal_t` | `<18,2>` | normals, rotation matrix | wraps (range is never exceeded by valid input) |
| `compute_t` | `<32,16>` | residual, Jacobian | saturates |
| `accum_t` | `<64,32>` | `H`, `g`, cost | wraps; 24 integer bits are needed in the worst case |
| `grid_t` | `<26,12>` | position in voxel units | saturates, always truncates so the index is an exact floor |

`coord_t` and `compute_t` round to nearest (`VOXLIO_QUANT = AP_RND`). The
host converts float inputs by rounding to nearest with saturation; the
testbench does the same.

## Decisions that differ from or extend the original design

1. **Grid origin shifted by half a voxel.** `MAP_*_MIN` is -8.25 / -8.25 /
   -2.25 instead of -8 / -8 / -2. With the original values the synthetic planes
   (z = 0, x = 4, y = 4) lie exactly on voxel faces, so range noise splits
   each plane across two voxel layers and each layer fits a biased plane. On
   the face-aligned case the pose error after five iterations is 6.1 mm
   instead of 0.93 mm. Extent, resolution and grid dimensions are unchanged.
2. **Bounds check on the scaled coordinate.** The original design tests `px < xmin` or
   `px >= xmax`. The code tests `0 <= (px - xmin) / voxel_size < NX`, which
   is the same condition in exact arithmetic. In float, `(px - xmin)` can
   round up to the far edge for a point just inside it; testing the value
   that is then truncated guarantees `0 <= ix < NX`.
3. **Status and edge-case semantics** as tabulated above; the original design lists the
   bits and cases but not their behaviour.
4. **Normal orientation.** The map builder makes the largest-magnitude
   component of each normal positive. A viewpoint-based rule is undefined when the reference origin lies on the plane, which is
   the case for the ground plane here. `H`, `g` and the cost do not depend on
   the sign.
5. **Scan generation.** Scan points are sampled directly on the planes on a
   lattice different from the map's and moved into the LiDAR frame with the
   ground-truth pose. There is no ray casting or occlusion model. About 1 %
   clutter points exercise the rejection paths.
6. **Interface pragmas.** `offset=slave` and separate bundles were added to
   the original interface pragmas.
7. **Fixed point is all-or-nothing.** `ap_fixed` and `float` do not mix in
   one expression, so a category cannot be switched alone. The sweep narrows
   one category while the others are held wide enough to be exact, which
   gives the same attribution.
8. **Extra directories.** `reference/cpp/` holds the phase 4 reference and
   `tests/` the Python tests; neither is in the original directory layout.
9. **Optional planarity gate.** `build_voxel_map(max_thickness_ratio=...)`
   can reject voxels whose points are not plane-like. It is off by default,
   so the builder matches the original algorithm.

## Not yet known

Nothing here has been through Vitis HLS. Whether the struct ports map to
AXI-Lite as written, how the 12-byte and 28-byte elements are padded on the
AXI masters, and all latency and resource figures are open until phases 6
and 7 are run.
