#!/usr/bin/env python3
"""Summarise Vitis HLS synthesis reports (verification level 5).

Reads every results/synthesis/<solution>/*csynth.xml left there by
hls/run_hls.tcl and writes results/synthesis/summary.md with the target and
estimated clock, latency, interval and resource estimates.

Only numbers found in a report are printed; a field the report does not
contain is shown as "n/a". Nothing is estimated here.

NOTE: written against the documented csynth.xml layout without access to the
Xilinx tools; check the first real summary against the .rpt file.
"""

import pathlib
import sys
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]
SYNTH = ROOT / "results" / "synthesis"

FIELDS = [
    ("tool version", "ReportVersion/Version"),
    ("part", "UserAssignments/Part"),
    ("target clock (ns)", "UserAssignments/TargetClockPeriod"),
    ("clock uncertainty (ns)", "UserAssignments/ClockUncertainty"),
    ("estimated clock (ns)", "PerformanceEstimates/SummaryOfTimingAnalysis/EstimatedClockPeriod"),
    ("latency best (cycles)", "PerformanceEstimates/SummaryOfOverallLatency/Best-caseLatency"),
    ("latency average (cycles)", "PerformanceEstimates/SummaryOfOverallLatency/Average-caseLatency"),
    ("latency worst (cycles)", "PerformanceEstimates/SummaryOfOverallLatency/Worst-caseLatency"),
    ("interval min (cycles)", "PerformanceEstimates/SummaryOfOverallLatency/Interval-min"),
    ("interval max (cycles)", "PerformanceEstimates/SummaryOfOverallLatency/Interval-max"),
    ("LUT", "AreaEstimates/Resources/LUT"),
    ("FF", "AreaEstimates/Resources/FF"),
    ("DSP", "AreaEstimates/Resources/DSP"),
    ("BRAM_18K", "AreaEstimates/Resources/BRAM_18K"),
    ("URAM", "AreaEstimates/Resources/URAM"),
    ("available LUT", "AreaEstimates/AvailableResources/LUT"),
    ("available FF", "AreaEstimates/AvailableResources/FF"),
    ("available DSP", "AreaEstimates/AvailableResources/DSP"),
    ("available BRAM_18K", "AreaEstimates/AvailableResources/BRAM_18K"),
]


def summarise(report):
    root = ET.parse(report).getroot()
    return {name: (root.findtext(path) or "n/a").strip() for name, path in FIELDS}


def main():
    reports = sorted(SYNTH.glob("*/*csynth.xml"))
    # Prefer the top-level report when a solution holds one per sub-function.
    top = [r for r in reports if r.name in ("csynth.xml", "voxlio_core_csynth.xml")]
    reports = top or reports
    if not reports:
        sys.exit(f"no csynth.xml under {SYNTH}; run `make hls` on a machine with Vitis HLS")

    columns = {report.parent.name: summarise(report) for report in reports}
    lines = [
        "# VoxLIO synthesis summary",
        "",
        "Vitis HLS C-synthesis estimates, copied from the csynth.xml reports; these are",
        "pre-implementation estimates, not post-route results. The settings of each",
        "solution are in its `config.txt`.",
        "",
        "| metric | " + " | ".join(columns) + " |",
        "|---|" + "---:|" * len(columns),
    ]
    for name, _ in FIELDS:
        lines.append(f"| {name} | " + " | ".join(col[name] for col in columns.values()) + " |")
    lines.append("")

    (SYNTH / "summary.md").write_text("\n".join(lines))
    print("\n".join(lines))


if __name__ == "__main__":
    main()
