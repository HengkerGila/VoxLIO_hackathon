#!/usr/bin/env python3
"""Compare result files and report the error metrics of each verification level.

    compare_outputs.py --level float [--report FILE] CASE...
    compare_outputs.py --level fixed [--report FILE] CASE...

Level "float" checks, per case:
    Python float32 golden model  vs  plain C++ reference      (acceptance test 8)
    plain C++ reference          vs  HLS-compatible C++ core  (acceptance test 9)
and reports the rounding error of single precision against the float64 run of
the golden model.

Level "fixed" checks the fixed-point core against the float C++ reference
(acceptance test 10).

Level "rtl" checks the Verilator result of the RTL against the fixed-point
C++ core (expected bit-exact) and against the float reference.

Exit status is non-zero if any comparison is outside its tolerance.
"""

import argparse
import datetime
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from reference.geometry import unpack_h              # noqa: E402
from reference.voxlio_reference import read_result   # noqa: E402

DATA = ROOT / "data" / "synthetic"
VERIF = ROOT / "results" / "verification"

# Tolerances. "rel_h" / "rel_g" are Frobenius / Euclidean norms of the error
# relative to the reference; "inliers" is the allowed difference in
# inlier_count as a fraction of the processed points; "update_rot" (rad) and
# "update_trans" (m) bound the difference between the two pose updates
# delta = solve(H, -g).
TOLERANCES = {
    # Same arithmetic in a different language: expected bit-exact, the bound
    # only absorbs a compiler that orders float operations differently.
    "exact": dict(rel_h=1e-6, rel_g=1e-6, inliers=0.0, update_rot=1e-6, update_trans=1e-6),
    # float32 vs float64 arithmetic on identical inputs.
    "single": dict(rel_h=1e-4, rel_g=1e-3, inliers=1e-3, update_rot=1e-5, update_trans=1e-5),
    # Selected fixed-point tolerance (acceptance test 10). With 1 cm range
    # noise the synthetic room determines the pose update to about 9e-5 rad
    # and 0.35 mm (1 sigma, norm over the three axes). Quantisation may add at
    # most 30 % of that, which inflates the total error by under 5 % when the
    # two add in quadrature.
    "fixed": dict(rel_h=1e-3, rel_g=1e-2, inliers=2e-3, update_rot=3e-5, update_trans=1e-4),
}


def pose_update(result):
    return np.linalg.solve(unpack_h(result.H), -result.g)


def metrics(ref, cand):
    """Error of `cand` against `ref` (level 4 metric list)."""
    H_ref, H_cand = unpack_h(ref.H), unpack_h(cand.H)
    d_ref, d_cand = pose_update(ref), pose_update(cand)
    return {
        "inliers": cand.inlier_count - ref.inlier_count,
        "counts_equal": (ref.inlier_count, ref.processed_count, ref.rejected_count, ref.status)
                        == (cand.inlier_count, cand.processed_count, cand.rejected_count, cand.status),
        "processed": ref.processed_count,
        "max_abs_h": float(np.max(np.abs(cand.H - ref.H))),
        "mean_abs_h": float(np.mean(np.abs(cand.H - ref.H))),
        "rel_h": float(np.linalg.norm(H_cand - H_ref) / np.linalg.norm(H_ref)),
        "max_abs_g": float(np.max(np.abs(cand.g - ref.g))),
        "rel_g": float(np.linalg.norm(cand.g - ref.g) / np.linalg.norm(ref.g)),
        "abs_cost": abs(cand.cost - ref.cost),
        "rel_cost": abs(cand.cost - ref.cost) / abs(ref.cost),
        "update_rot": float(np.linalg.norm(d_cand[:3] - d_ref[:3])),
        "update_trans": float(np.linalg.norm(d_cand[3:] - d_ref[3:])),
    }


def within(m, tol):
    return (abs(m["inliers"]) <= tol["inliers"] * m["processed"]
            and m["rel_h"] <= tol["rel_h"] and m["rel_g"] <= tol["rel_g"]
            and m["update_rot"] <= tol["update_rot"] and m["update_trans"] <= tol["update_trans"])


COLUMNS = [
    ("inliers", "inlier diff", "{:+d}"),
    ("max_abs_h", "max abs H", "{:.3g}"),
    ("mean_abs_h", "mean abs H", "{:.3g}"),
    ("rel_h", "rel H", "{:.3g}"),
    ("max_abs_g", "max abs g", "{:.3g}"),
    ("rel_g", "rel g", "{:.3g}"),
    ("rel_cost", "rel cost", "{:.3g}"),
    ("update_rot", "update rot (rad)", "{:.3g}"),
    ("update_trans", "update trans (m)", "{:.3g}"),
]


def comparisons(level, case):
    """(label, reference file, candidate file, tolerance name, gating)."""
    data, out = DATA / case, VERIF / case
    if level == "float":
        return [
            ("Python f32 vs C++ reference", data / "expected_f32.txt", out / "cpp_ref.txt", "exact", True),
            ("C++ reference vs HLS float core", out / "cpp_ref.txt", out / "hls_float.txt", "exact", True),
            ("Python f64 vs Python f32", data / "expected_f64.txt", data / "expected_f32.txt", "single", True),
        ]
    if level == "fixed":
        return [
            ("C++ float reference vs HLS fixed core", out / "cpp_ref.txt", out / "hls_fixed.txt", "fixed", True),
        ]
    return [
        ("C++ fixed core vs RTL", out / "hls_fixed.txt", out / "rtl_fixed.txt", "exact", True),
        ("C++ float reference vs RTL", out / "cpp_ref.txt", out / "rtl_fixed.txt", "fixed", True),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--level", choices=["float", "fixed", "rtl"], required=True)
    parser.add_argument("--report", type=pathlib.Path, help="also write a Markdown report here")
    parser.add_argument("--note", default="", help="free text added to the report header")
    parser.add_argument("cases", nargs="+")
    args = parser.parse_args()

    header = "| case | comparison | " + " | ".join(c[1] for c in COLUMNS) + " | result |"
    rows = [header, "|" + "---|" * (len(COLUMNS) + 3)]
    failed = False

    for case in args.cases:
        for label, ref_path, cand_path, tol_name, gating in comparisons(args.level, case):
            m = metrics(read_result(ref_path), read_result(cand_path))
            ok = within(m, TOLERANCES[tol_name])
            exact = m["counts_equal"] and m["max_abs_h"] == 0 and m["max_abs_g"] == 0 and m["abs_cost"] == 0
            verdict = ("bit-exact" if exact else "pass") if ok else "FAIL"
            failed |= gating and not ok
            cells = [fmt.format(m[key]) for key, _, fmt in COLUMNS]
            rows.append(f"| {case} | {label} | " + " | ".join(cells) + f" | {verdict} |")

    table = "\n".join(rows)
    print(table)

    if args.report:
        used = {"float": ["exact", "single"], "fixed": ["fixed"], "rtl": ["exact", "fixed"]}[args.level]
        tolerance_lines = [
            f"- `{name}`: " + ", ".join(f"{k} <= {v:g}" for k, v in TOLERANCES[name].items())
            for name in used
        ]
        text = [
            f"# VoxLIO verification report: {args.level}",
            "",
            f"Generated by `scripts/compare_outputs.py` on {datetime.date.today().isoformat()}."
            + (f" {args.note}" if args.note else ""),
            "",
            table,
            "",
            "Errors are of the second operand against the first. `rel H` is the Frobenius norm of",
            "the 6 x 6 error over that of the reference; `update` is the difference between the",
            "pose updates `solve(H, -g)` computed from each result.",
            "",
            "Tolerances:",
            *tolerance_lines,
            "",
        ]
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text("\n".join(text))

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
