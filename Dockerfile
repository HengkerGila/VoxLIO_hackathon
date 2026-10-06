# Reproducible environment for `make test` (Python, C++, fixed point, RTL).
# Quartus is not included; run `make quartus` on a machine that has it.
#
#   docker build -t voxlio .
#   docker run --rm -it -v "$PWD":/work voxlio make test

FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential git ca-certificates \
        python3 python3-numpy python3-pytest \
        verilator iverilog \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY Makefile ./
RUN make third_party/ap_types/include/ap_fixed.h

CMD ["make", "check"]
