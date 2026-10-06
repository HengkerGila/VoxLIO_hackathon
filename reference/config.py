"""Constants shared with the hardware model.

Everything here is parsed from hls/voxlio_config.hpp, which is the single
place where tunables are edited.
"""

import pathlib
import re

HEADER = pathlib.Path(__file__).resolve().parents[1] / "hls" / "voxlio_config.hpp"

_DEFINE = re.compile(r"\s*#\s*define\s+(\w+)\s+(\S+)\s*(//.*)?$")
_INT = re.compile(r"[-+]?(0[xX][0-9a-fA-F]+|\d+)[uU]?$")
_FLOAT = re.compile(r"[-+]?(\d+\.\d*|\.\d+|\d+)([eE][-+]?\d+)?[fF]?$")


def parse_defines(path=HEADER):
    """Return {name: value} for every single-literal numeric #define."""
    values = {}
    for line in path.read_text().splitlines():
        m = _DEFINE.match(line)
        if not m:
            continue
        name, token = m.group(1), m.group(2)
        if _INT.match(token):
            values.setdefault(name, int(token.rstrip("uU"), 0))
        elif _FLOAT.match(token):
            values.setdefault(name, float(token.rstrip("fF")))
    return values


_d = parse_defines()

MAX_POINTS = _d["MAX_POINTS"]

NX = _d["NX"]
NY = _d["NY"]
NZ = _d["NZ"]
NUM_VOXELS = NX * NY * NZ
GRID_DIMS = (NX, NY, NZ)

VOXEL_SIZE = _d["VOXEL_SIZE"]
MAP_MIN = (_d["MAP_X_MIN"], _d["MAP_Y_MIN"], _d["MAP_Z_MIN"])

RESIDUAL_THRESHOLD = _d["RESIDUAL_THRESHOLD"]
NEIGHBOR_RADIUS = _d["NEIGHBOR_RADIUS"]

STATUS_SUCCESS = _d["VOXLIO_STATUS_SUCCESS"]
STATUS_ZERO_INLIERS = _d["VOXLIO_STATUS_ZERO_INLIERS"]
STATUS_BAD_MAP_CONFIG = _d["VOXLIO_STATUS_BAD_MAP_CONFIG"]
STATUS_BAD_POSE = _d["VOXLIO_STATUS_BAD_POSE"]
STATUS_POINT_OVERFLOW = _d["VOXLIO_STATUS_POINT_OVERFLOW"]
STATUS_OOB_PREVENTED = _d["VOXLIO_STATUS_OOB_PREVENTED"]

FIXED_FORMATS = {
    name: (_d[f"VOXLIO_{name}_W"], _d[f"VOXLIO_{name}_I"])
    for name in ("COORD", "NORMAL", "COMPUTE", "ACCUM")
}
FIXED_QUANT = re.search(r"#\s*define\s+VOXLIO_QUANT\s+(AP_\w+)", HEADER.read_text()).group(1)
