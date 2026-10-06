#!/bin/bash
# Install everything `make test` needs on Ubuntu 24.04 (or Debian 12+), then
# fetch the ap_fixed headers. Quartus is the one tool this script cannot
# install; see README.md.
#
#   scripts/setup_ubuntu.sh            # apt + pip --user + headers
#   scripts/setup_ubuntu.sh --venv     # same, but Python packages in ./.venv

set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)

APT_PACKAGES="build-essential git python3 python3-pip python3-venv verilator iverilog"

if ! command -v apt-get >/dev/null; then
    echo "This script is for apt-based systems. Install by hand: $APT_PACKAGES, then pip install -r requirements.txt"
    exit 1
fi

echo "== apt packages"
sudo apt-get update
sudo apt-get install -y $APT_PACKAGES

echo "== python packages"
if [ "${1:-}" = "--venv" ]; then
    python3 -m venv "$REPO/.venv"
    "$REPO/.venv/bin/pip" install -r "$REPO/requirements.txt"
    echo "activate with: source .venv/bin/activate"
else
    pip3 install --user --break-system-packages -r "$REPO/requirements.txt" 2>/dev/null \
        || pip3 install --user -r "$REPO/requirements.txt"
fi

echo "== ap_fixed headers (Apache 2.0, pinned revision)"
make -C "$REPO" third_party/ap_types/include/ap_fixed.h

echo "== check"
make -C "$REPO" check
