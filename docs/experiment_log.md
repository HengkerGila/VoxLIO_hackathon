# Experiment log

## 2026-10-05: first implementation pass

Machine: Intel Core i3-10110U, Ubuntu, g++ 13.3.0, Python 3.12, numpy 1.26.
No Vitis HLS. All numbers come from the generated reports under `results/`;
rerun the named `make` target to reproduce them.

Scene unless stated otherwise: case `room`, 13,950 scan points, 564 valid
voxels, 1 cm Gaussian range noise on map and scan, predicted pose 87.5 mm and
1.118 degrees from the ground truth.

### 1. Float equivalence (`make float`)

Python float32, plain C++ and the HLS-compatible core agree in every bit of
`H`, `g`, cost, counters and status on all four cases. Of the 13,950 points,
13,811 are inliers; 85 fall outside the grid, 48 have no valid neighbour and
6 exceed the residual threshold. A point sees 26.5 in-grid candidates on
average, of which 9.4 are valid.

float32 against float64 arithmetic on the same inputs: relative `H` error up
to 9.3e-6, pose update moved by up to 3.5e-7 rad and 2.3e-6 m, same inlier
count. Single-precision accumulation over about 14,000 points is not a
concern at this scan size.

### 2. Grid origin and voxel faces (`make bench`)

Distance to the ground-truth pose after each host iteration, translation in
mm / rotation in degrees
([pose_convergence.md](../results/benchmark/pose_convergence.md)):

| Setup | iter 1 | iter 2 | iter 5 |
|---|---|---|---|
| Planes at voxel centres | 14.89 / 0.149 | 6.18 / 0.057 | 0.93 / 0.007 |
| Planes on voxel faces | 28.78 / 0.291 | 15.88 / 0.143 | 6.13 / 0.056 |

With the original example origin (-8, -8, -2) and scene (z = 0, x = 4, y = 4),
every plane sits on a voxel face. Noise then splits each plane between two
voxel layers, the number of valid voxels doubles (1,090 against 564) and each
layer fits a plane offset to its own side. The first correction is half as
good and convergence is several times slower. **Decision:** shift the grid
origin by half a voxel (0.25 m). The face-aligned scene is kept as a test
case.

This is a property of the synthetic scene. In real data surfaces cross voxel
faces everywhere, so the face-aligned row is the more representative one for
what a fixed grid will see.

### 3. Planarity gate in the map builder

Voxels along the floor-wall edges hold points from two planes and PCA returns
a diagonal normal for them. Rejecting voxels whose smallest covariance
eigenvalue exceeds 5 % of the middle one removes 44 of 564 voxels and gives
12.63 mm / 0.076 degrees after one iteration instead of 14.89 mm / 0.149
degrees, and 0.62 mm instead of 0.93 mm after five. **Decision:** keep the
gate available but off, because the original builder algorithm has no such
gate. Turning it on is a host-side change only.

### 4. The minimum-residual rule slows convergence

With the 27-voxel search the pose error roughly halves per iteration instead
of collapsing after one or two, which is unusual for Gauss-Newton on clean
planes. Restricting the search to the point's own voxel (neighbour radius 0)
on the same scene gives:

| Search | inliers | iter 1 | iter 2 | iter 5 |
|---|---|---|---|---|
| 3 x 3 x 3, minimum residual | 13,811 | 14.89 / 0.149 | 6.18 / 0.057 | 0.93 / 0.007 |
| Own voxel only | 13,806 | 3.78 / 0.043 | 0.42 / 0.010 | 0.51 / 0.008 |

The own-voxel search is four times closer after one iteration, settles in
two, loses five inliers, and needs one memory read per point instead of 27.
The likely reason: a point near a plane sees about nine voxels of that plane,
each with a slightly different fitted plane, and taking the smallest
`|residual|` among them systematically under-estimates the distance. The
measurement supports this but does not isolate it.

On the face-aligned scene the own-voxel search is not clearly better (4.30 mm
/ 0.081 degrees after five iterations against 6.13 mm / 0.056 degrees), so
this is not a general result.

**Not changed:** the 3 x 3 x 3 neighbourhood and the minimum-residual rule
are frozen for v0.1. This is the experimental evidence
that section asks for before changing them; a rule such as "own voxel first,
neighbours only if it is invalid" is the obvious next thing to try. It bears
directly on question Q6 and on throughput.

### 5. Fixed point (`make fixed`, `make sweep`)

Method: every category `ap_fixed`, one category narrowed at a time while the
others are held wide enough to be exact
([fixed_point_sweep.md](../results/verification/fixed_point_sweep.md)).

- With all categories wide, the fixed-point core reproduces the float64
  golden model to 1e-14 rad and 3e-12 m, so the fixed-point code path
  computes the same thing as the float one.
- Integer bits needed for this map: 5 for coordinates, 2 for normals, 5 for
  the Jacobian, 24 for `H` at 32,768 points. The configured 10 / 2 / 16 / 32
  have margin; coordinates also have to hold LiDAR-frame points, whose range
  depends on the sensor.
- Narrowed alone, with truncation, each category needs 14 fraction bits (16
  for the accumulators) to stay inside the tolerance. With rounding the
  intermediates need only 10.
- **Truncation bias.** `ap_fixed` truncates by default, which shifts every
  transformed coordinate and every residual by half an LSB in the same
  direction. With the baseline widths this moves the pose update by up
  to 1.0e-4 m. Rounding (`AP_RND`) brings that to 4.9e-5 m at the same
  widths. **Decision:** `VOXLIO_QUANT = AP_RND`, widths left at the baseline
  values. Wider formats (`<26,10>` coordinates, `<20,2>` normals) reach
  1e-5 m but no longer fit one 18-bit multiplier input; that trade needs
  synthesis numbers.
- **Tolerance.** The pose update from this scene has a 1-sigma uncertainty of
  about 9.4e-5 rad and 0.35 mm (from `sigma^2 * inverse(H)` with sigma =
  1 cm). The first gate tried was 10 % of that. It turned out to sit at the
  size of a discrete effect: quantisation changes which of several nearly
  tied candidates wins for a few points (18 of 13,950 in a Python emulation
  of 14-fraction-bit coordinate truncation), and the error is therefore not
  monotonic in width. A gate that tight tests luck with ties as much as
  precision. **Decision:** 30 % of 1 sigma, that is 3e-5 rad and 1e-4 m,
  which inflates the total error by under 5 % when the two add in
  quadrature. Under the 10 % gate the default configuration would fail on
  rotation (1.4e-5 rad against 1e-5).
- Result with the default configuration on the four cases: inlier counts
  identical to float, pose update within 1.4e-5 rad and 4.9e-5 m.

Not done: resource comparison between float and fixed point (needs
synthesis).

### 6. CPU baseline (`make bench`)

The plain C++ reference, one thread, `-O2`, median of 200 runs
([cpu_reference.md](../results/benchmark/cpu_reference.md)):

| Case | points | valid candidates/point | ms per scan | ns per point |
|---|---|---|---|---|
| `room` | 13,950 | 9.4 | 1.85 | 133 |
| `room_face_aligned` | 14,918 | 18.0 | 2.66 | 178 |

Consequence for the hardware baseline: the v0.1 search evaluates candidates
one after another, 27 per point, so it cannot take fewer than 27 clock cycles
per point. At 100 MHz that is 270 ns per point and 3.8 ms for this scan,
about twice the laptop CPU's time, before any memory-interface latency. This
is a bound derived from the architecture, not a measurement. The baseline
should be expected to be slower than a desktop-class CPU, and the fair
comparison for a first result is the embedded processor it would replace.

### 7. Host transfer (question Q8)

By construction the float build returns 28 accumulator words and 4 counters,
128 bytes per scan, whatever the scan size. Returning the residual and
Jacobian of every inlier instead would be 7 floats for each of 13,811 points,
387 kB for this scan.

## 2026-10-06: RTL for the DE10-Nano

The board is an Intel Cyclone V, so the AMD HLS path was replaced by
hand-written SystemVerilog (`rtl/`, `docs/rtl.md`).

### 8. RTL equivalence (`make rtl`)

The RTL reproduces the fixed-point C++ core bit for bit on the four vectors,
30 directed runs and 8 random scenarios of 2,000 points. Because the
C++ fixed-point core is itself checked against the float reference and the
Python golden model, the chain Python -> C++ float -> C++ fixed -> RTL is
closed. Nine planted RTL bugs are all caught by the testbench; one of them
(a non-strict threshold compare) survived until a case landing exactly on
the threshold was added.

### 9. Cycle counts (Verilator)

| Point outcome | Cycles |
|---|---|
| inlier | 48 |
| rejected after the search | 43 |
| rejected by the bounds check | 8 |
| `room`, 13,950 points | 665,932 total, 47.7 per point |

27 of the 48 cycles are the sequential candidate walk; about 14 are pipeline
fill and drain between stages because one point is in flight at a time, and
the rest are the state machine. At 100 MHz the scan takes 6.7 ms, against
1.85 ms for the C++ reference on the laptop (entry 6); the clock the core
reaches and the Cortex-A9 timing are both unmeasured. Overlapping
consecutive points is the first optimisation; it does not change the
algorithm.

### Open questions

| Question | State |
|---|---|
| Q1 Latency from voxel lookup | needs synthesis |
| Q2 Latency from geometry arithmetic | needs synthesis |
| Q3 Achievable point throughput | 47.7 cycles per point measured in RTL simulation (entry 9); clock frequency needs synthesis |
| Q4 Does fixed point reduce resources | needs synthesis |
| Q5 Numerical error of quantisation | measured, entry 5 |
| Q6 Is the 27-neighbour search necessary | on the centred synthetic room, no: the own-voxel search is more accurate (entry 4); not general |
| Q7 On-chip map memory feasible | the RTL keeps map (1.04 Mbit) and scan buffer (2.36 Mbit) in M10K, 60 % of the Cyclone V SE by arithmetic; the fit needs Quartus |
| Q8 Does the H/g reduction cut host transfer | yes by construction, entry 7 |
