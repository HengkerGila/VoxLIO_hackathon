# Verification

State on 2026-10-05. Everything below runs with `make test` unless marked
otherwise. Reports are regenerated into `results/` on every run.

## Levels

| Level | What is compared | How | State |
|---|---|---|---|
| 1 Unit tests | each stage against hand-computed values | `tests/test_geometry.py`, `tests/test_voxel_map.py`, `testbench/tb_{transform,voxel_index,geometry,accumulator}.cpp` | pass |
| 2 Python vs C++ | golden model (float32) vs `voxlio_ref.cpp` | `tb_voxlio`, `compare_outputs.py --level float` | bit-exact on 4 cases |
| 3 HLS C simulation | `voxlio_ref.cpp` vs `voxlio_core` | same testbench, compiled with g++ | bit-exact on 4 cases; **Vitis csim not run** |
| 4 Float vs fixed point | float reference vs `ap_fixed` core | `make fixed`, `make sweep` | inside tolerance on 4 cases (g++ simulation) |
| RTL | fixed-point C++ core vs `rtl/*.sv` in Verilator | `make rtl`, `compare_outputs.py --level rtl` | bit-exact on 4 cases, 30 directed and 8 random runs |
| 5 Synthesis | clock, resources | `make quartus` | **not run** |

## Acceptance tests

| Test | Python | C++ | State |
|---|---|---|---|
| 1 Single point on a known plane | `test_1_single_point_known_residual` | `tb_voxlio: test_1_...` | pass, exact |
| 2 Point exactly on the plane | `test_2_point_on_plane_has_zero_residual` | `tb_voxlio: test_2_...`, `tb_geometry` | pass |
| 3 Point outside the map | `test_3_point_outside_map_is_rejected` | `tb_voxlio: test_3_...`, `tb_voxel_index` | pass |
| 4 Boundary voxel | `test_4_boundary_voxel_is_matched`, `..._does_not_alias` | `tb_voxlio: test_4_boundary_voxels` under AddressSanitizer | pass |
| 5 All neighbours invalid | `test_5_all_neighbours_invalid_is_rejected` | `tb_voxlio: test_5_...` | pass |
| 6 Known Jacobian | `test_jacobian_manual_case` | `tb_geometry` | pass, exact |
| 7 Two constraints | `test_7_two_constraints_accumulate` | `tb_accumulator`, `tb_voxlio: test_two_constraints` | pass, exact |
| 8 Room scan, Python == C++ | exported by `export_vectors.py` | `tb_voxlio` part 2 | bit-exact, 4 cases |
| 9 Room scan, C++ == HLS simulation | | `tb_voxlio` part 2 | bit-exact under g++; Vitis csim pending |
| 10 Fixed point within tolerance | | `tb_voxlio` fixed build, `compare_outputs.py --level fixed` | pass, 4 cases |

Test 4 has two parts. A plane in a corner voxel must be matched normally.
Then the only valid voxel is placed where an unchecked neighbour address
would land (the wrapped row, address -1, address `NUM_VOXELS`) with a plane
through the query point, so a missing bounds check turns a rejection into an
inlier; the C++ build also runs under AddressSanitizer.

The defined edge cases are covered in `tests/test_reference_model.py`
and `tb_voxlio`; their defined behaviour is tabulated in
[architecture.md](architecture.md).

## Test vectors

`scripts/export_vectors.py` writes four cases to `data/synthetic/`:

| Case | Purpose |
|---|---|
| `room` | The reference scene: ground z = 0, walls x = 4 and y = 4, 13,950 points. |
| `room_face_aligned` | The same room moved by half a voxel, so the planes lie on voxel faces. |
| `room_seed_b`, `room_seed_c` | `room` with other noise seeds and other prediction errors. |

Each case holds the scan, the voxel map, the predicted pose, and the golden
model's output in float32 and float64. The testbench refuses vectors whose
recorded grid configuration differs from the compiled one.

## Tolerances

Defined in `scripts/compare_outputs.py`.

| Name | Used for | Bound |
|---|---|---|
| `exact` | Python f32 vs C++, C++ vs HLS float | counters equal; relative `H` and `g` error <= 1e-6. Observed: 0. |
| `single` | float32 vs float64 golden model | relative `H` <= 1e-4, pose update within 1e-5 rad and 1e-5 m. Observed: 9.3e-6, 3.5e-7 rad, 2.3e-6 m. |
| `fixed` | fixed-point core vs float reference | pose update within 3e-5 rad and 1e-4 m, relative `H` <= 1e-3, relative `g` <= 1e-2, inlier count within 0.2 %. Observed worst: 1.4e-5 rad, 4.9e-5 m, 2.7e-4, 7.7e-4, 0. |

The `fixed` bound is 30 % of the pose update's own 1-sigma uncertainty at
1 cm range noise; see the experiment log for why it is not tighter.

Bit-exact agreement of the float builds relies on three things: every
expression is written in the same order in all three implementations, the
C++ is built with `-ffp-contract=off` so no multiply-add is fused, and the
Python model uses numpy float32 scalars throughout. A Vitis HLS run could
still differ if the tool reorders float operations; that is what level 3
with the real tool will show.

## Do the tests detect bugs?

`make mutation` plants 30 single-line bugs, one at a time, in a scratch copy
and reruns the relevant test: 21 in the C++ and Python (removed bounds
checks, flipped comparison operators, Jacobian and accumulator sign errors, a
wrong packing-table entry, skipped overflow and pose checks, wrapping instead
of saturating fixed point) and 9 in the RTL (truncation instead of rounding,
non-strict comparisons, a dropped bounds check, a Jacobian sign, accumulator
packing, a dropped zero-normal check, disabled saturation). All 30 are
caught, and the unmutated controls pass. One RTL mutant, the non-strict
threshold, survived the first run until a directed case that lands exactly
on the threshold was added; the earlier cases only reached one coordinate
LSB on either side.

## Known gaps

- **No synthesis tool has seen this code.** The RTL is lint-clean in
  Verilator and parses in Icarus, but Quartus has not run; level 5 is open.
- **`NX == NY`.** With a square grid, swapping `NX` and `NY` in the address
  formula changes nothing, so no test can detect that mistake. A regression
  with a non-square grid would close this.
- **Synthetic data only.** No real LiDAR frame has been run (milestone M6).
- **Interface level.** The RTL is driven through its raw ports; there is no
  bus wrapper or host driver yet.
- **Fixed point.** The fixed-point build is simulated with the open-source
  `ap_fixed` headers under g++, not with the headers shipped in Vitis HLS.
  Accumulator overflow is argued from ranges (24 integer bits needed, 32
  provided), not tested at the worst case.
