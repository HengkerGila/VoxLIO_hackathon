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

This is the interface of the HLS function. The hand-written RTL has plain
ports instead ([rtl.md](rtl.md)); how the host will reach them on the
DE10-Nano is specified in [host_interface.md](host_interface.md).

## Data definitions

Every piece of data the core reads, hands from one stage to the next and
returns, with its type and its width in bits. The widths are those of the
fixed-point build, which is the one the RTL implements. In the float build
each of the five number types is a 32-bit IEEE float instead. Types and
structs are declared in `hls/voxlio_types.hpp`, constants in
`hls/voxlio_config.hpp`; `rtl/voxlio_pkg.sv` is generated from the latter.

### Number types

| Type | Bits | Fraction bits | One LSB | Range | Holds |
|---|---:|---:|---|---|---|
| `coord_t` | 24 | 14 | `2^-14` m = 61 µm | ±512 m | scan points, centroids, pose translation |
| `normal_t` | 18 | 16 | `2^-16` = 1.5e-5 | ±2 | plane normals, rotation matrix |
| `compute_t` | 32 | 16 | `2^-16` = 1.5e-5 | ±32768 | residual, Jacobian |
| `accum_t` | 64 | 32 | `2^-32` | ±2.1e9 | `H`, `g`, cost |
| `grid_t` | 26 | 14 | `2^-14` voxel | ±2048 voxels | a position in voxel units, before it is truncated to an index |

All five are signed two's complement. The host converts a real value `v` to
the raw integer `round(v * 2^F)`, saturated to the range, where `F` is the
number of fraction bits: 0.125 m as `coord_t` is 2048 (`0x000800`), a normal
component of 1.0 as `normal_t` is 65536 (`0x10000`), and the 0.30 m threshold
as `compute_t` is 19661. How each type rounds and overflows inside the core
is under [Numeric modes](#numeric-modes).

### Sizes

| Constant | Value | Gives |
|---|---|---|
| `MAX_POINTS` | 32768 | scan buffer depth; a 15-bit point address |
| `NX`, `NY`, `NZ` | 32, 32, 8 | voxel coordinates `ix`, `iy`, `iz` of 5, 5 and 3 bits |
| `NUM_VOXELS` | 8192 | voxel map depth; a 13-bit voxel address |
| `VOXEL_SIZE` | 0.5 m | a grid of 16 x 16 x 4 m |
| `MAP_X_MIN`, `MAP_Y_MIN`, `MAP_Z_MIN` | -8.25, -8.25, -2.25 m | minimum corner of the grid |
| `RESIDUAL_THRESHOLD` | 0.30 m | inlier threshold |
| `NEIGHBOR_RADIUS` | 1 | 27 candidate voxels per point |

### Interface data

What crosses between the host and the core, as declared for `voxlio_core()`:

| Data | Type | Fields | Bits, fixed point | Bytes, float | How many, direction |
|---|---|---|---:|---:|---|
| Scan point | `Point3D` | `x`, `y`, `z`: `coord_t` | 72 | 12 | up to `MAX_POINTS`, host to core |
| Voxel entry | `VoxelEntry` | `cx`, `cy`, `cz`: `coord_t`; `nx`, `ny`, `nz`: `normal_t`; `valid`: flag | 127 | 28 | `NUM_VOXELS`, host to core |
| Pose | `Pose3D` | `R[9]`: `normal_t`, row-major; `t[3]`: `coord_t` | 234 | 48 | one, host to core |
| Point count | `uint32_t` | `num_points` | 32 | 4 | one, host to core |
| Result | `VoxLIOResult` | `H[21]`, `g[6]`, `cost`: `accum_t`; `inlier_count`, `processed_count`, `rejected_count`, `status`: `uint32_t` | 1920 | 128 | one, core to host |

The pose is 162 bits of rotation and 72 of translation. The result is 1344
bits of `H`, 384 of `g`, 64 of cost and four 32-bit words. `H` is the upper
triangle packed by columns ([Conventions](#conventions)); the bits of
`status` are listed under
[Status word and edge cases](#status-word-and-edge-cases). `valid` is a
`uint8_t` in C++, where any non-zero value means valid, and a single bit in
the RTL word. In the float build `VoxelEntry` is 25 bytes of fields padded
to 28.

The fixed-point column is the packed width, which is what the RTL uses. How
Vitis HLS pads the structs on its AXI ports is not known yet
([Not yet known](#not-yet-known)).

### Memory words and ports in the RTL

The RTL keeps the scan and the map in two RAMs inside the core
([rtl.md](rtl.md)). The host fills them through one write port each:
`scan_waddr` (15 bits) with `scan_wdata`, and `map_waddr` (13 bits) with
`map_wdata`. Fields are packed with the first field in the least significant
bits.

Scan buffer: 32768 words of 72 bits, 2.36 Mbit. Word `i` is point `i`.

| Bits | Field | Type |
|---|---|---|
| 23:0 | `x` | `coord_t` |
| 47:24 | `y` | `coord_t` |
| 71:48 | `z` | `coord_t` |

Voxel map: 8192 words of 127 bits, 1.04 Mbit. Word `k` is voxel `k`, with
`k = iz * NX * NY + iy * NX + ix`. Because `NX` and `NY` are powers of two,
that is the bit concatenation `{iz, iy, ix}` (3 + 5 + 5 bits).

| Bits | Field | Type |
|---|---|---|
| 23:0 | `cx` | `coord_t` |
| 47:24 | `cy` | `coord_t` |
| 71:48 | `cz` | `coord_t` |
| 89:72 | `nx` | `normal_t` |
| 107:90 | `ny` | `normal_t` |
| 125:108 | `nz` | `normal_t` |
| 126 | `valid` | 1 bit |

The pose, the point count and the result are plain ports, not memories:

| Port | Bits | Layout |
|---|---:|---|
| `pose_r` | 162 | `R` row-major, element `i` in bits `18*i +: 18` |
| `pose_t` | 72 | `t`, element `i` in bits `24*i +: 24` |
| `num_points` | 32 | unsigned |
| `h_flat` | 1344 | packed `H[k]` in bits `64*k +: 64` |
| `g_flat` | 384 | `g[i]` in bits `64*i +: 64` |
| `cost` | 64 | `accum_t` |
| `inlier_count`, `processed_count`, `rejected_count`, `status` | 32 each | unsigned |

### Data between stages

These symbols label the arrows of the flowchart in
[mathematics.md](mathematics.md#data-flow). Names in brackets are the RTL
signals.

| Data | Symbol | From | To | Type | Bits |
|---|---|---|---|---|---:|
| Scan point | `q` | scan buffer | stage 1 | 3 x `coord_t` | 72 |
| Pose (`pose_r`, `pose_t`) | `R`, `t` | host | stage 1 | 9 x `normal_t`, 3 x `coord_t` | 162 + 72 |
| Map-frame point (`px`, `py`, `pz`) | `p` | stage 1 | stages 2, 3, 6, 9 | 3 x `coord_t` | 72 |
| Grid coordinate | `f` | inside stages 2 and 3 | | 3 x `grid_t` | 78 |
| Voxel index (`ix`, `iy`, `iz`) | `i` | stage 3 | stage 4 | unsigned | 5 + 5 + 3 |
| Candidate address (`map_raddr`) | `k` | stage 4 | voxel map | unsigned | 13 |
| Voxel entry (`map_rdata`) | `c_k`, `n_k`, `valid_k` | voxel map | stages 5, 6 | map word | 127 |
| Candidate residual | `r_k` | stage 6 | stage 7 | `compute_t` | 32 |
| Best residual (`best_r`) | `r` | stage 7 | stages 8, 10, 11 | `compute_t` | 32 |
| Best normal (`best_nx`, `best_ny`, `best_nz`) | `n` | stage 7 | stage 9 | 3 x `normal_t` | 54 |
| Jacobian (`j_flat`) | `J` | stage 9 | stage 11 | 6 x `compute_t` | 192 |
| Normal equations (`h_flat`, `g_flat`, `cost`) | `H`, `g`, cost | stage 11 | host | 28 x `accum_t` | 1792 |
| Counters and status | | stage 11, control | host | 4 x `uint32_t` | 128 |

Three one-bit flags steer the control: in grid (stage 2), found (stage 7,
tested in stage 8) and inlier (stage 10). In the C++ model `ix`, `iy` and
`iz` are plain `int`, and stage 7 hands on the whole selected `VoxelEntry`;
the RTL keeps only its normal, which is all stage 9 uses.

### Widths inside the arithmetic

A product or sum of stored values is kept exact and is narrowed once, where
the result is stored. The intermediate widths below are the RTL's:

| Stage | Exact intermediate | Bits | Stored as |
|---|---|---:|---|
| 1 Point transform | `R * q`, one product | 42 | |
| | three products plus `t` | 45 | `coord_t`: 16 bits dropped, rounded, saturated |
| 2, 3 Bounds check, voxel index | `p - m` | 25 | |
| | times `1 / s` | 49 | `grid_t`: 14 bits dropped, truncated, saturated |
| 6 Plane candidate evaluation | `p - c` | 25 | |
| | `n * (p - c)`, one product | 43 | |
| | sum of three products | 45 | `compute_t`: 14 bits dropped, rounded, saturated |
| 9 Jacobian | `p * n`, one product | 42 | |
| | difference of two products | 43 | `compute_t`: 14 bits dropped, rounded, saturated |
| 11 Accumulation | `J * J`, `J * r`, `r * r` | 44, 40, 36 | added into 64-bit `accum_t`, which wraps |

The accumulator multiplies the low 22 bits of each Jacobian entry and the
low 18 bits of the residual, which loses nothing for any value that can
reach it ([rtl.md](rtl.md#fixed-point-mapping)).

### Test vector files

`scripts/export_vectors.py` writes each case to `data/synthetic/<case>/`.
The inputs are little-endian binary; the fixed-point testbenches convert the
values when they load them, rounding to nearest and saturating.

| File | Element | Shape | Content |
|---|---|---|---|
| `scan.bin` | float32 | (`num_points`, 3) | scan points, LiDAR frame |
| `voxel_desc.bin` | float32 | (8192, 6) | `cx cy cz nx ny nz` per voxel |
| `voxel_valid.bin` | uint8 | (8192,) | valid flag per voxel |
| `pose.bin` | float32 | (12,) | predicted `R` row-major, then `t` |
| `expected_f32.txt`, `expected_f64.txt` | text | 32 lines | golden result: the four counters, then `H 0..20`, `g 0..5` and `cost` as C99 hex floats |
| `meta.txt` | text | `key value` lines | grid configuration, `num_points`, scene and diagnostics; the testbench refuses a case whose grid differs from the build |

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
