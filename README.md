# VoxLIO v0.1

A streaming voxel-based hardware accelerator for the geometric front-end of
LiDAR scan-to-map localization. For every scan point it transforms the point
with the predicted pose, searches the 3 x 3 x 3 voxel neighbourhood for the
best local plane, and accumulates the point-to-plane normal equations. Only
`H` (21 values), `g` (6), the cost and three counters go back to the host,
which solves the 6 x 6 system.

[docs/architecture.md](docs/architecture.md) describes the design as built and
[docs/mathematics.md](docs/mathematics.md) explains the computation of each
stage with a flowchart. The target board is a DE10-Nano (Intel Cyclone V
SoC), so the deliverable core is hand-written SystemVerilog under `rtl/`,
described in [docs/rtl.md](docs/rtl.md); the HLS C++ under `hls/` is the
bit-accurate model it is verified against.

## Status

| Phase | State | Evidence |
|---|---|---|
| 1. Repository bootstrap | done | this tree |
| 2. Synthetic generator | done | `reference/generate_synthetic.py`, four seeded cases in `data/synthetic/` |
| 3. Python golden model | done | 76 tests in `tests/` pass |
| 4. Plain C++ reference | done | bit-exact against the Python float32 model on all four cases |
| 5. HLS-compatible C++ | done, compiled with g++ only | bit-exact against the C++ reference; no STL, recursion or dynamic memory |
| 6. HLS C simulation | replaced | the board is Intel; `hls/run_hls.tcl` stays for an AMD port but was never run |
| 7. First synthesis | **not run** | Quartus project in `quartus/`, written without the tool; no clock or resource number exists yet |
| 8. Fixed point | numeric half done | error measured with g++ and the open-source `ap_fixed` headers; resource half needs synthesis |
| RTL (replaces 5 to 7 for the DE10-Nano) | done, simulated | `rtl/*.sv`, bit-exact against the fixed-point C++ core in Verilator; 47.7 cycles per point |
| 9. Optimisation | not started | by design, waits for a synthesis baseline |

Milestones M0 to M2 are complete and M3 holds for the RTL (cycle-accurate
simulation equals the C++ model). M4 needs a Quartus run.

## Getting started

```sh
scripts/setup_ubuntu.sh   # Ubuntu 24.04 / Debian 12: apt packages, Python packages, ap_fixed headers
make check                # says what is installed, what is missing, and whether make test can run
make test                 # Python tests, float build, fixed-point build, RTL simulation, all comparisons
```

Any other OS: `docker build -t voxlio .` then `docker run --rm -it -v "$PWD":/work voxlio make test`.

What `make test` needs: Python 3 with numpy and pytest (`requirements.txt`),
g++ with C++14, make, git (the fixed-point build clones the Apache-licensed
`ap_fixed` headers into `third_party/`), and Verilator 5 for the RTL
simulation (Ubuntu 22.04 ships 4.x, which is too old; use 24.04, the Docker
image, or oss-cad-suite). Quartus Prime Lite is the one manual install and
is only needed for `make quartus`.

Other targets:

```sh
make rtl        # Verilator: RTL against the fixed-point C++ core, bit for bit
make quartus    # synthesis and fit for the DE10-Nano; needs quartus_sh on the PATH
make bench      # CPU baseline timing and the host-loop convergence experiment
make sweep      # fixed-point width sweep (about 10 minutes)
make mutation   # plant 30 bugs (21 software, 9 RTL) and confirm the tests catch each one
```

## Results so far

All figures are from the synthetic room: a ground plane and two walls with
1 cm range noise, about 14,000 scan points.

- **Equivalence.** Python float32, plain C++ and the HLS-compatible C++ core
  give identical `H`, `g`, cost, counters and status on every case
  ([float_report.md](results/verification/float_report.md)).
- **Single precision.** float32 against float64 arithmetic moves the pose
  update by at most 3.5e-7 rad and 2.3e-6 m.
- **Localization.** One host solve reduces the pose error from 87.5 mm /
  1.118 degrees to 14.9 mm / 0.149 degrees; five iterations reach 0.93 mm /
  0.007 degrees ([pose_convergence.md](results/benchmark/pose_convergence.md)).
- **Fixed point.** With the baseline widths (coordinates `<24,10>`,
  normals `<18,2>`, intermediates `<32,16>`, accumulators `<64,32>`) and
  rounding, the pose update differs from the float baseline by at most
  1.4e-5 rad and 0.05 mm, inside the selected tolerance of 3e-5 rad and
  0.1 mm. With truncation the same widths miss it on one case
  ([fixed_point_sweep.md](results/verification/fixed_point_sweep.md)).
- **RTL.** The SystemVerilog core produces bit-identical `H`, `g`, cost,
  counters and status to the fixed-point C++ core on every vector, directed
  and random scenario ([rtl_report.md](results/verification/rtl_report.md)),
  and takes 665,932 cycles for the 13,950-point `room` scan: 47.7 cycles per
  point, 6.7 ms at 100 MHz.
- **CPU baseline.** The plain C++ reference takes 1.85 ms for the same scan
  on one core of an i3-10110U, about 133 ns per point
  ([cpu_reference.md](results/benchmark/cpu_reference.md)).

There is no clock frequency or resource count in this repository, because
nothing has been synthesized.

## Points that need a decision or a follow-up

1. **Run Quartus.** `make quartus` on the machine that has Quartus Lite. The
   project files and `scripts/summarize_quartus.py` were written without the
   tool and have never been executed; expect to fix small things.
2. **The baseline is slower than a laptop CPU, as expected.** 47.7 cycles per
   point measured in simulation: 27 are the sequential candidate walk that
   v0.1 prescribes, about 14 are pipeline bubbles from having one point in
   flight. The fair comparison is the DE10-Nano's Cortex-A9, not yet
   measured. Overlapping points removes the bubbles; the walk itself is the
   subject of experiment log entry 4.
3. **Grid origin.** Shifted by half a voxel from the original example; see
   decision 1 in [docs/architecture.md](docs/architecture.md). It is one line
   in `hls/voxlio_config.hpp` to revert.
4. **Fixed-point tolerance.** The gate (30 % of the estimator's own 1-sigma
   noise) is a choice; the reasoning is in
   [docs/experiment_log.md](docs/experiment_log.md).
5. **Host interface.** The RTL exposes plain write ports and registers; the
   Avalon-MM wrapper for the HPS bridge and the Linux-side driver are not
   written. The on-chip scan buffer takes 2.4 Mbit of M10K at
   `MAX_POINTS = 32768`; streaming points from the HPS instead is the
   alternative if the fit is tight.

## Layout

```text
hls/          HLS-style C++ core; voxlio_config.hpp holds every tunable
rtl/          SystemVerilog core for the DE10-Nano (voxlio_pkg.sv is generated)
quartus/      Quartus Lite project for the Cyclone V SE
reference/    Python golden model, map builder, synthetic generator
reference/cpp plain C++ float reference (phase 4)
testbench/    C++ unit and top-level testbenches, Verilator testbench, CPU benchmark
tests/        Python tests
scripts/      vector export, comparison, sweep, benchmark, mutation check
data/         generated test vectors
results/      generated reports
docs/         as-built architecture, mathematics, RTL, verification, experiment log
```
