#!/usr/bin/env python3
"""Fixed-point width exploration.

Builds the fixed-point core with different ap_fixed formats, runs the
synthetic test vectors through it and measures the error against the float
baseline. Each numeric category is narrowed on its own while the other three
are held at formats wide enough to be exact, so the error of each
quantisation can be read off separately. A few complete candidate
configurations are then measured on every case.

Writes results/verification/fixed_point_sweep.md and .csv.
Needs the float results (run `make float` first) and the ap_fixed headers.
"""

import concurrent.futures
import csv
import datetime
import os
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from compare_outputs import TOLERANCES, metrics, within   # noqa: E402
from reference import config as cfg                       # noqa: E402
from reference.voxlio_reference import read_result        # noqa: E402

BUILD = ROOT / "build" / "sweep"
DATA = ROOT / "data" / "synthetic"
VERIF = ROOT / "results" / "verification"
AP_TYPES_INC = pathlib.Path(os.environ.get("AP_TYPES_INC", ROOT / "third_party" / "ap_types" / "include"))

SOURCES = ["testbench/tb_voxlio.cpp", "hls/transform.cpp", "hls/voxel_index.cpp",
           "hls/voxel_lookup.cpp", "hls/geometry.cpp", "hls/accumulator.cpp",
           "hls/voxlio_core.cpp", "reference/cpp/voxlio_ref.cpp"]

CATEGORIES = ["COORD", "NORMAL", "COMPUTE", "ACCUM"]

# Formats (W, I) wide enough that the category adds no measurable error.
# "QUANT" is the quantisation mode of coord_t and compute_t.
WIDE = {"COORD": (48, 10), "NORMAL": (40, 2), "COMPUTE": (64, 16), "ACCUM": (96, 40),
        "QUANT": "AP_TRN"}

# Fractional bits tried per category; integer bits stay at the configured value.
FRACTION_SWEEP = {
    "COORD": [6, 8, 10, 12, 14, 16, 18, 20],
    "NORMAL": [8, 10, 12, 14, 16, 18, 20, 22],
    "COMPUTE": [6, 8, 10, 12, 14, 16, 18, 20, 24],
    "ACCUM": [4, 8, 12, 16, 20, 24, 32],
}

# Complete configurations measured on every case. "baseline" is the
# starting set of widths; "config" is whatever hls/voxlio_config.hpp holds.
def _full(coord, normal, compute, accum=(64, 32), quant="AP_TRN"):
    return {"COORD": coord, "NORMAL": normal, "COMPUTE": compute, "ACCUM": accum, "QUANT": quant}


CANDIDATES = {
    "baseline widths": _full((24, 10), (18, 2), (32, 16)),
    "baseline widths, rounded": _full((24, 10), (18, 2), (32, 16), quant="AP_RND"),
    "coord 26 normal 20": _full((26, 10), (20, 2), (32, 16)),
    "coord 26 normal 20, rounded": _full((26, 10), (20, 2), (32, 16), quant="AP_RND"),
    "coord 27 normal 22 compute 36": _full((27, 10), (22, 2), (36, 16)),
    "config (voxlio_config.hpp)": {**cfg.FIXED_FORMATS, "QUANT": cfg.FIXED_QUANT},
}

CASES = ["room", "room_face_aligned", "room_seed_b", "room_seed_c"]


def tag_of(formats):
    widths = "_".join(f"{c[0]}{formats[c][0]}.{formats[c][1]}" for c in CATEGORIES)
    return f"{widths}_{formats['QUANT']}"


def run_config(formats):
    """Build and run one configuration; returns {case: result}.

    Results are reused when they are newer than every source file.
    """
    out = BUILD / tag_of(formats)
    outputs = [out / "out" / case / "hls_fixed.txt" for case in CASES]
    inputs = [ROOT / src for src in SOURCES] + list((ROOT / "hls").glob("*.hpp")) + [
        ROOT / "testbench" / "tb_util.hpp", *(DATA / c / "scan.bin" for c in CASES)]
    newest_input = max(path.stat().st_mtime for path in inputs)
    if all(path.exists() and path.stat().st_mtime > newest_input for path in outputs):
        return {case: read_result(path) for case, path in zip(CASES, outputs)}

    for case in CASES:
        (out / "out" / case).mkdir(parents=True, exist_ok=True)
    defines = [f"-DVOXLIO_{c}_{k}={v}" for c in CATEGORIES for k, v in zip("WI", formats[c])]
    defines.append(f"-DVOXLIO_QUANT={formats['QUANT']}")
    binary = out / "tb_voxlio"
    subprocess.run(
        ["g++", "-std=c++14", "-O1", "-w", "-ffp-contract=off", "-DVOXLIO_FIXED_POINT",
         f"-I{AP_TYPES_INC}", "-Ihls", "-Ireference/cpp", "-Itestbench", *defines,
         *SOURCES, "-o", str(binary)],
        cwd=ROOT, check=True)
    # The testbench's own coarse gate fails for narrow formats; only the
    # result files matter here.
    subprocess.run([str(binary), str(out / "out"), *(str(DATA / c) for c in CASES)],
                   cwd=ROOT, stdout=subprocess.DEVNULL, check=False)
    return {case: read_result(out / "out" / case / "hls_fixed.txt") for case in CASES}


def bits_for(max_abs):
    """Integer bits, sign included, needed to hold +/- max_abs."""
    bits = 1
    while 2 ** (bits - 1) <= max_abs:
        bits += 1
    return bits


def read_meta(case):
    meta = {}
    for line in (DATA / case / "meta.txt").read_text().splitlines():
        key, _, value = line.partition(" ")
        try:
            meta[key] = float(value)
        except ValueError:
            pass
    return meta


def main():
    if not (AP_TYPES_INC / "ap_fixed.h").exists():
        sys.exit(f"ap_fixed.h not found in {AP_TYPES_INC}; run `make fixed` once or set AP_TYPES_INC")
    baseline = {}
    for case in CASES:
        path = VERIF / case / "cpp_ref.txt"
        if not path.exists():
            sys.exit(f"{path} missing; run `make float` first")
        baseline[case] = (read_result(path), read_result(DATA / case / "expected_f64.txt"))

    jobs = {}  # (kind, label, case-independent) -> formats
    for category, fractions in FRACTION_SWEEP.items():
        integer = cfg.FIXED_FORMATS[category][1]
        for fraction in fractions:
            formats = dict(WIDE)
            formats[category] = (integer + fraction, integer)
            jobs[("sweep", category, fraction)] = formats
    jobs[("wide", "all wide", 0)] = dict(WIDE)
    for category in ("COORD", "COMPUTE"):
        integer = cfg.FIXED_FORMATS[category][1]
        for fraction in FRACTION_SWEEP[category]:
            formats = dict(WIDE, QUANT="AP_RND")
            formats[category] = (integer + fraction, integer)
            jobs[("sweep_rnd", category, fraction)] = formats
    for name, formats in CANDIDATES.items():
        jobs[("candidate", name, 0)] = formats

    unique = {tag_of(f): f for f in jobs.values()}
    print(f"building and running {len(unique)} configurations ...")
    with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count()) as pool:
        results = dict(zip(unique, pool.map(run_config, unique.values())))

    tol = TOLERANCES["fixed"]
    rows = []

    def measure(kind, label, formats, case):
        result = results[tag_of(formats)][case]
        m32 = metrics(baseline[case][0], result)
        m64 = metrics(baseline[case][1], result)
        row = {
            "kind": kind, "label": label, "case": case,
            **{f"{c.lower()}": f"<{formats[c][0]},{formats[c][1]}>" for c in CATEGORIES},
            "quant": formats["QUANT"],
            "inlier_diff": m32["inliers"], "rel_h": m32["rel_h"], "rel_g": m32["rel_g"],
            "update_rot": m32["update_rot"], "update_trans": m32["update_trans"],
            "update_rot_f64": m64["update_rot"], "update_trans_f64": m64["update_trans"],
            "pass": within(m32, tol),
        }
        rows.append(row)
        return row

    def line(cells):
        return "| " + " | ".join(str(c) for c in cells) + " |"

    def fmt(row):
        return [f"{row['inlier_diff']:+d}", f"{row['rel_h']:.2e}", f"{row['rel_g']:.2e}",
                f"{row['update_rot']:.2e}", f"{row['update_trans']:.2e}",
                f"{row['update_rot_f64']:.2e}", f"{row['update_trans_f64']:.2e}",
                "pass" if row["pass"] else "fail"]

    metric_head = ["inlier diff", "rel H", "rel g", "update rot (rad)", "update trans (m)",
                   "rot vs f64", "trans vs f64", "tolerance"]

    text = [
        "# Fixed-point width sweep",
        "",
        f"Generated by `scripts/fixed_point_sweep.py` on {datetime.date.today().isoformat()}"
        " (g++ simulation with the ap_fixed headers; no synthesis).",
        "",
        "Errors are against the float32 C++ baseline unless marked `vs f64` (the float64",
        "golden model). `update` is the difference between the pose updates `solve(H, -g)`.",
        "The float32 baseline itself differs from float64 by about 3e-7 rad and 2e-6 m, so",
        "differences of that size are at the noise floor of the comparison.",
        "",
        "The error does not fall monotonically with width. Besides the smooth rounding error,",
        "quantisation can change which of several nearly tied candidate voxels wins the",
        "minimum-residual search for a few points, and each such change swaps that point's",
        "constraint for a neighbouring plane's. Rows whose `rel H` sits near 2e-4 to 4e-4 while",
        "their neighbours are an order of magnitude lower are consistent with such changes.",
        "",
        "Selected tolerance (acceptance test 10): " + ", ".join(f"{k} <= {v:g}" for k, v in tol.items()) + ".",
        "",
        "## Integer bits",
        "",
        "Largest magnitudes seen on the test vectors and the worst case that the map",
        f"geometry allows (a point anywhere in the grid, {cfg.MAX_POINTS} inliers).",
        "",
        line(["category", "quantity", "observed max", "worst case", "bits needed (signed)", "configured"]),
        line(["---"] * 6),
    ]

    metas = [read_meta(case) for case in CASES]
    extent = [abs(m) for m in cfg.MAP_MIN] + [
        abs(m + n * cfg.VOXEL_SIZE) for m, n in zip(cfg.MAP_MIN, cfg.GRID_DIMS)]
    max_coord = max(extent)
    corner = sum(max(abs(m), abs(m + n * cfg.VOXEL_SIZE)) ** 2
                 for m, n in zip(cfg.MAP_MIN, cfg.GRID_DIMS)) ** 0.5
    worst_h = cfg.MAX_POINTS * corner ** 2
    observed = lambda key: max(m[key] for m in metas)   # noqa: E731
    range_rows = [
        ("coord_t", "map-frame coordinate", observed("max_abs_point"), max_coord, "COORD"),
        ("normal_t", "normal / rotation element", 1.0, 1.0, "NORMAL"),
        ("compute_t", "Jacobian entry |p x n|", observed("max_abs_jacobian"), corner, "COMPUTE"),
        ("accum_t", "H entry", observed("max_abs_h"), worst_h, "ACCUM"),
    ]
    for name, quantity, seen, worst, category in range_rows:
        text.append(line([name, quantity, f"{seen:.4g}", f"{worst:.4g}", bits_for(worst),
                          cfg.FIXED_FORMATS[category][1]]))
    text += [
        "",
        "coord_t also carries LiDAR-frame scan points, whose range is set by the sensor and",
        "not by the map; out-of-range values saturate and are then rejected as outside the grid.",
        "",
    ]

    wide_formats = jobs[("wide", "all wide", 0)]
    text += ["## All categories wide", "",
             "Sanity check of the method: with every format wide the fixed-point core must",
             "reproduce the float64 result.", "",
             line(["case", "formats (coord, normal, compute, accum)", *metric_head]),
             line(["---"] * (2 + len(metric_head)))]
    for case in CASES:
        row = measure("wide", "all wide", wide_formats, case)
        text.append(line([case, " ".join(row[c.lower()] for c in CATEGORIES), *fmt(row)]))
    text.append("")

    for kind, mode in (("sweep", "truncating (AP_TRN)"), ("sweep_rnd", "rounding (AP_RND)")):
        for category, fractions in FRACTION_SWEEP.items():
            if (kind, category, fractions[0]) not in jobs:
                continue
            others = ", ".join(f"{c.lower()}_t <{WIDE[c][0]},{WIDE[c][1]}>" for c in CATEGORIES if c != category)
            text += [f"## {category.lower()}_t alone, {mode}", "",
                     f"Case `room`; other categories held wide ({others}).", "",
                     line(["format", "fraction bits", *metric_head]),
                     line(["---"] * (2 + len(metric_head)))]
            for fraction in fractions:
                row = measure(kind, category, jobs[(kind, category, fraction)], "room")
                text.append(line([row[category.lower()], fraction, *fmt(row)]))
            text.append("")

    text += ["## Complete configurations", "",
             line(["name", "case", "coord", "normal", "compute", "accum", "quantisation", *metric_head]),
             line(["---"] * (7 + len(metric_head)))]
    for name, formats in CANDIDATES.items():
        for case in CASES:
            row = measure("candidate", name, formats, case)
            text.append(line([name, case, *(row[c.lower()] for c in CATEGORIES), formats["QUANT"], *fmt(row)]))
    text.append("")

    VERIF.mkdir(parents=True, exist_ok=True)
    (VERIF / "fixed_point_sweep.md").write_text("\n".join(text))
    with open(VERIF / "fixed_point_sweep.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print("\n".join(text))


if __name__ == "__main__":
    main()
