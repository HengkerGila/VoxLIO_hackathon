// Verilator testbench for rtl/voxlio_core.sv.
//
// Every scenario is run through the fixed-point C++ core (hls/ built with
// -DVOXLIO_FIXED_POINT) and through the RTL, and the two results must agree
// bit for bit: all 21 H entries, g, cost, the three counters and the status
// word. The C++ fixed-point core is itself checked against the float
// reference and the Python golden model by tb_voxlio, so this closes the
// chain Python -> C++ float -> C++ fixed -> RTL.
//
// Usage: tb_rtl [output_dir case_dir...]
// Directed and random scenarios always run; test vectors are run when given
// and their RTL results are written to output_dir/<case>/rtl_fixed.txt.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "Vvoxlio_core.h"
#include "verilated.h"

#include "tb_util.hpp"
#include "voxlio_core.hpp"
#include "voxlio_ref.hpp"

#ifndef VOXLIO_FIXED_POINT
#error "tb_rtl must be built with -DVOXLIO_FIXED_POINT"
#endif

namespace {

// ---- Raw bit access ----------------------------------------------------------------

template <typename T>
int64_t raw_bits(const T &x) {
    ap_int<T::width> bits;
    bits.range(T::width - 1, 0) = x.range(T::width - 1, 0);
    return bits.to_int64();
}

uint64_t mask_bits(int width) { return width >= 64 ? ~0ull : ((1ull << width) - 1); }

int64_t sign_extend(uint64_t v, int width) {
    if (width >= 64) return (int64_t)v;
    const uint64_t sign = 1ull << (width - 1);
    return (int64_t)((v ^ sign) - sign);
}

// Verilator wide signals are arrays of 32-bit words, word 0 = bits [31:0].
template <typename W>
void set_bits(W &w, int offset, int width, uint64_t value) {
    for (int b = 0; b < width; ++b) {
        const int bit = offset + b;
        const uint32_t m = 1u << (bit % 32);
        if ((value >> b) & 1) {
            w[bit / 32] |= m;
        } else {
            w[bit / 32] &= ~m;
        }
    }
}

template <typename W>
uint64_t get_bits(const W &w, int offset, int width) {
    uint64_t v = 0;
    for (int b = 0; b < width; ++b) {
        const int bit = offset + b;
        v |= (uint64_t)((w[bit / 32] >> (bit % 32)) & 1u) << b;
    }
    return v;
}

// ---- Results -----------------------------------------------------------------------

struct Raw {
    int64_t H[21];
    int64_t g[6];
    int64_t cost;
    uint32_t inlier_count, processed_count, rejected_count, status;
};

Raw raw_of(const VoxLIOResult &r) {
    Raw out;
    for (int k = 0; k < 21; ++k) out.H[k] = raw_bits(r.H[k]);
    for (int i = 0; i < 6; ++i) out.g[i] = raw_bits(r.g[i]);
    out.cost = raw_bits(r.cost);
    out.inlier_count = r.inlier_count;
    out.processed_count = r.processed_count;
    out.rejected_count = r.rejected_count;
    out.status = r.status;
    return out;
}

tb::ResultD to_result_d(const Raw &r) {
    // accum_t has ACCUM_W - ACCUM_I fraction bits.
    const double lsb = 1.0 / std::ldexp(1.0, VOXLIO_ACCUM_W - VOXLIO_ACCUM_I);
    tb::ResultD d;
    for (int k = 0; k < 21; ++k) d.H[k] = (double)r.H[k] * lsb;
    for (int i = 0; i < 6; ++i) d.g[i] = (double)r.g[i] * lsb;
    d.cost = (double)r.cost * lsb;
    d.inlier_count = r.inlier_count;
    d.processed_count = r.processed_count;
    d.rejected_count = r.rejected_count;
    d.status = r.status;
    return d;
}

bool same(const Raw &a, const Raw &b, const char *name) {
    bool ok = true;
    for (int k = 0; k < 21; ++k) {
        if (a.H[k] != b.H[k]) {
            if (ok) std::printf("  %s: H[%d] C++ %lld RTL %lld\n", name, k, (long long)a.H[k], (long long)b.H[k]);
            ok = false;
        }
    }
    for (int i = 0; i < 6; ++i) {
        if (a.g[i] != b.g[i]) {
            if (ok) std::printf("  %s: g[%d] C++ %lld RTL %lld\n", name, i, (long long)a.g[i], (long long)b.g[i]);
            ok = false;
        }
    }
    if (a.cost != b.cost) {
        if (ok) std::printf("  %s: cost C++ %lld RTL %lld\n", name, (long long)a.cost, (long long)b.cost);
        ok = false;
    }
    if (a.inlier_count != b.inlier_count || a.processed_count != b.processed_count ||
        a.rejected_count != b.rejected_count || a.status != b.status) {
        if (ok)
            std::printf("  %s: counts C++ %u/%u/%u status %u, RTL %u/%u/%u status %u\n", name,
                        a.inlier_count, a.processed_count, a.rejected_count, a.status,
                        b.inlier_count, b.processed_count, b.rejected_count, b.status);
        ok = false;
    }
    return ok;
}

// ---- RTL driver --------------------------------------------------------------------

const int CW = VOXLIO_COORD_W;
const int NW = VOXLIO_NORMAL_W;
const int AW = VOXLIO_ACCUM_W;

struct Rtl {
    std::unique_ptr<VerilatedContext> ctx;
    std::unique_ptr<Vvoxlio_core> top;
    uint64_t cycles = 0;

    Rtl(int argc, char **argv) : ctx(new VerilatedContext) {
        ctx->commandArgs(argc, argv);
        top.reset(new Vvoxlio_core(ctx.get()));
        top->clk = 0;
        top->rst_n = 0;
        top->scan_we = 0;
        top->map_we = 0;
        top->start = 0;
        top->num_points = 0;
        tick();
        tick();
        top->rst_n = 1;
        tick();
    }

    void tick() {
        top->clk = 0;
        top->eval();
        top->clk = 1;
        top->eval();
        ++cycles;
    }

    void write_scan(uint32_t i, const Point3D &p) {
        set_bits(top->scan_wdata, 0 * CW, CW, (uint64_t)raw_bits(p.x));
        set_bits(top->scan_wdata, 1 * CW, CW, (uint64_t)raw_bits(p.y));
        set_bits(top->scan_wdata, 2 * CW, CW, (uint64_t)raw_bits(p.z));
        top->scan_waddr = i;
        top->scan_we = 1;
        tick();
        top->scan_we = 0;
    }

    void write_map(uint32_t k, const VoxelEntry &v) {
        set_bits(top->map_wdata, 0 * CW, CW, (uint64_t)raw_bits(v.cx));
        set_bits(top->map_wdata, 1 * CW, CW, (uint64_t)raw_bits(v.cy));
        set_bits(top->map_wdata, 2 * CW, CW, (uint64_t)raw_bits(v.cz));
        set_bits(top->map_wdata, 3 * CW + 0 * NW, NW, (uint64_t)raw_bits(v.nx));
        set_bits(top->map_wdata, 3 * CW + 1 * NW, NW, (uint64_t)raw_bits(v.ny));
        set_bits(top->map_wdata, 3 * CW + 2 * NW, NW, (uint64_t)raw_bits(v.nz));
        set_bits(top->map_wdata, 3 * CW + 3 * NW, 1, v.valid != 0);
        top->map_waddr = k;
        top->map_we = 1;
        tick();
        top->map_we = 0;
    }

    void set_pose(const Pose3D &pose) {
        for (int i = 0; i < 9; ++i) set_bits(top->pose_r, i * NW, NW, (uint64_t)raw_bits(pose.R[i]));
        for (int i = 0; i < 3; ++i) set_bits(top->pose_t, i * CW, CW, (uint64_t)raw_bits(pose.t[i]));
    }

    // Returns false on timeout.
    bool run(uint32_t num_points, Raw &out, uint64_t &run_cycles) {
        top->num_points = num_points;
        top->start = 1;
        tick();
        top->start = 0;
        const uint64_t begin = cycles;
        const uint64_t limit = 200ull * (uint64_t)(num_points <= MAX_POINTS ? num_points : 1) + 1000;
        while (!top->done) {
            tick();
            if (cycles - begin > limit) {
                return false;
            }
        }
        run_cycles = cycles - begin;
        for (int k = 0; k < 21; ++k) out.H[k] = sign_extend(get_bits(top->h_flat, k * AW, AW), AW);
        for (int i = 0; i < 6; ++i) out.g[i] = sign_extend(get_bits(top->g_flat, i * AW, AW), AW);
        out.cost = (int64_t)top->cost;
        out.inlier_count = top->inlier_count;
        out.processed_count = top->processed_count;
        out.rejected_count = top->rejected_count;
        out.status = top->status;
        return true;
    }
};

// ---- Scenario state ------------------------------------------------------------------

Point3D g_scan[MAX_POINTS];
VoxelEntry g_map[NUM_VOXELS];
Rtl *g_rtl = 0;

int flat(int ix, int iy, int iz) { return (iz * NY + iy) * NX + ix; }
float cx(int ix) { return tb::voxel_centre(ix, MAP_X_MIN); }
float cy(int iy) { return tb::voxel_centre(iy, MAP_Y_MIN); }
float cz(int iz) { return tb::voxel_centre(iz, MAP_Z_MIN); }

void clear_map() {
    for (int k = 0; k < NUM_VOXELS; ++k) g_map[k] = tb::voxel(0, 0, 0, 0, 0, 0, 0);
}

void set_plane(int ix, int iy, int iz, float px, float py, float pz, float nx, float ny, float nz,
               uint8_t valid = 1) {
    g_map[flat(ix, iy, iz)] = tb::voxel(px, py, pz, nx, ny, nz, valid);
}

void ground_map(float z = 0.0f) {
    clear_map();
    set_plane(16, 16, 4, 0, 0, z, 0, 0, 1);
}

void set_point(uint32_t i, float x, float y, float z) { g_scan[i] = tb::point(x, y, z); }

// Run C++ and RTL on the current scan/map; CHECK that they agree.
uint64_t compare(const char *name, uint32_t num_points, const float R[9] = tb::IDENTITY_R,
                 const float t[3] = tb::ZERO_T, Raw *keep = 0) {
    const Pose3D pose = tb::pose(R, t);

    VoxLIOResult cpp;
    voxlio_core(g_scan, num_points, g_map, pose, cpp);
    const Raw a = raw_of(cpp);

    for (int k = 0; k < NUM_VOXELS; ++k) g_rtl->write_map(k, g_map[k]);
    const uint32_t stored = num_points <= MAX_POINTS ? num_points : MAX_POINTS;
    for (uint32_t i = 0; i < stored; ++i) g_rtl->write_scan(i, g_scan[i]);
    g_rtl->set_pose(pose);

    Raw b;
    uint64_t run_cycles = 0;
    const bool finished = g_rtl->run(num_points, b, run_cycles);
    CHECK(finished);
    if (!finished) {
        std::printf("  %s: RTL did not finish\n", name);
        return 0;
    }
    CHECK(same(a, b, name));
    if (keep) *keep = b;
    return run_cycles;
}

// ---- Directed scenarios -------------------------------------------------------------

void directed() {
    // Acceptance tests 1, 2, 7: known constraints.
    ground_map();
    set_point(0, 0.1f, -0.05f, 0.125f);
    set_point(1, -0.2f, 0.15f, -0.0625f);
    Raw r;
    compare("two constraints", 2, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.inlier_count == 2 && r.status == VOXLIO_STATUS_SUCCESS);
    set_point(0, 0.2f, 0.1f, 0.0f);
    compare("point on plane", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.cost == 0);

    // Test 3: outside the map.
    set_point(0, 100.0f, 0.0f, 0.0f);
    compare("outside map", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.status == (VOXLIO_STATUS_ZERO_INLIERS | VOXLIO_STATUS_OOB_PREVENTED));

    // Test 4: boundary voxels and aliasing traps.
    const int corners[4][3] = {{0, 0, 0}, {NX - 1, NY - 1, NZ - 1}, {0, NY - 1, 3}, {NX - 1, 0, 0}};
    for (int c = 0; c < 4; ++c) {
        const int *v = corners[c];
        clear_map();
        set_plane(v[0], v[1], v[2], cx(v[0]), cy(v[1]), cz(v[2]), 0, 0, 1);
        set_point(0, cx(v[0]), cy(v[1]), cz(v[2]) + 0.0625f);
        compare("corner voxel", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
        CHECK(r.inlier_count == 1);
    }
    const int traps[4][2][3] = {{{0, 1, 0}, {NX - 1, 0, 0}},
                                {{NX - 1, 0, 0}, {0, 1, 0}},
                                {{0, 0, 0}, {NX - 1, NY - 1, NZ - 1}},
                                {{NX - 1, NY - 1, NZ - 1}, {0, 0, 0}}};
    for (int c = 0; c < 4; ++c) {
        const int *centre = traps[c][0];
        const int *trap = traps[c][1];
        clear_map();
        set_plane(trap[0], trap[1], trap[2], cx(centre[0]), cy(centre[1]), cz(centre[2]), 0, 0, 1);
        set_point(0, cx(centre[0]), cy(centre[1]), cz(centre[2]));
        compare("alias trap", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
        CHECK(r.inlier_count == 0 && r.rejected_count == 1);
    }

    // Test 5: neighbours present but none valid.
    ground_map();
    g_map[flat(16, 16, 4)].valid = 0;
    set_point(0, 0.0f, 0.0f, 0.05f);
    compare("all neighbours invalid", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.inlier_count == 0);

    // Plane selection: smaller |r| wins; ties keep the first in dz, dy, dx order.
    ground_map(0.20f);
    set_plane(17, 16, 4, 0.5f, 0.0f, -0.05f, 0, 0, 1);
    set_point(0, 0, 0, 0);
    compare("minimum residual", 1);
    ground_map(0.125f);
    set_plane(17, 16, 4, 0.5f, 0.0f, -0.125f, 0, 0, 1);
    compare("tie keeps first", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.g[5] < 0);
    const int reach[4][4] = {{17, 17, 5, 1}, {15, 15, 3, 1}, {18, 16, 4, 0}, {16, 16, 6, 0}};
    for (int c = 0; c < 4; ++c) {
        clear_map();
        set_plane(reach[c][0], reach[c][1], reach[c][2], 0, 0, 0, 0, 0, 1);
        compare("neighbour reach", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
        CHECK(r.inlier_count == (uint32_t)reach[c][3]);
    }

    // Threshold is strict, checked one raw LSB on each side. With a unit
    // normal the residual is (p.z - c.z) in coord_t units, i.e. a multiple of
    // 4 compute_t LSBs; the threshold raw value is 19661.
    set_point(0, 0, 0, 0);
    ground_map(-4915.0f / 16384.0f);     // r = 19660 < 19661: inlier
    compare("just below threshold", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.inlier_count == 1);
    ground_map(-4916.0f / 16384.0f);     // r = 19664: rejected
    compare("just above threshold", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.inlier_count == 0);
    // Exactly on the threshold: (p.z - c.z) = 0.25 m (raw 4096) times a normal
    // of raw 78644 gives 4096 * 78644 / 2^14 = 19661 exactly; must be rejected.
    clear_map();
    set_plane(16, 16, 4, 0, 0, -0.25f, 0, 0, 78644.0f / 65536.0f);
    compare("exactly at threshold", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.inlier_count == 0 && r.rejected_count == 1);
    set_plane(16, 16, 4, 0, 0, -0.25f, 0, 0, 78640.0f / 65536.0f);   // r = 19660: inlier
    compare("one LSB below threshold", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.inlier_count == 1);

    // Point counts.
    ground_map();
    compare("zero points", 0, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.status == VOXLIO_STATUS_ZERO_INLIERS);
    set_point(0, 0.0f, 0.0f, 0.125f);
    compare("overflow", MAX_POINTS + 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.status == (VOXLIO_STATUS_ZERO_INLIERS | VOXLIO_STATUS_POINT_OVERFLOW) && r.processed_count == 0);
    compare("overflow max", 0xFFFFFFFFu, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.processed_count == 0);

    // Empty map, zero-length normal, and a zero normal next to a real plane.
    clear_map();
    set_point(0, 0, 0, 0);
    set_point(1, 1, 1, 0);
    compare("empty map", 2);
    set_plane(16, 16, 4, 0, 0, 0, 0, 0, 0);
    set_point(0, 0.0f, 0.0f, 0.0625f);
    compare("zero normal", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.inlier_count == 0);
    set_plane(17, 16, 4, 0.5f, 0, 0, 0, 0, 1);
    compare("zero normal beside plane", 1, tb::IDENTITY_R, tb::ZERO_T, &r);
    CHECK(r.inlier_count == 1);

    // Saturation: 511 + 511 m must not wrap into the map.
    clear_map();
    set_plane(12, 16, 4, -2.0f, 0, 0, 0, 0, 1);
    set_point(0, 511.0f, 0, 0);
    const float t_far[3] = {511.0f, 0, 0};
    compare("saturating transform", 1, tb::IDENTITY_R, t_far, &r);
    CHECK(r.inlier_count == 0 && (r.status & VOXLIO_STATUS_OOB_PREVENTED));

    // A general pose.
    const float R[9] = {0.9992386f, -0.0350504f, -0.0171366f, 0.0348942f, 0.9993475f,
                        -0.0093303f, 0.0174524f, 0.0087252f, 0.9998096f};
    const float t[3] = {0.2f, -0.1f, 0.05f};
    ground_map();
    set_plane(24, 16, 4, 4.0f, 0.0f, 0.0f, 1, 0, 0);
    set_plane(16, 24, 4, 0.0f, 4.0f, 0.0f, 0, 1, 0);
    set_point(0, 0.3f, -0.2f, -0.04f);
    set_point(1, 3.7f, 0.1f, 0.2f);
    set_point(2, 0.4f, 3.95f, 0.3f);
    compare("rotated pose", 3, R, t, &r);
    CHECK(r.inlier_count == 3);
}

// ---- Random scenarios --------------------------------------------------------------

void random_scenarios(int runs, uint32_t points_per_run) {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    std::normal_distribution<float> gauss(0.0f, 1.0f);

    for (int run = 0; run < runs; ++run) {
        // Map: most voxels hold a random plane through a random point of the
        // voxel; a few are flagged invalid or carry a zero normal.
        for (int iz = 0; iz < NZ; ++iz) {
            for (int iy = 0; iy < NY; ++iy) {
                for (int ix = 0; ix < NX; ++ix) {
                    const float u = unit(rng);
                    float nx = gauss(rng), ny = gauss(rng), nz = gauss(rng);
                    const float len = std::sqrt(nx * nx + ny * ny + nz * nz) + 1e-9f;
                    nx /= len;
                    ny /= len;
                    nz /= len;
                    const float px = cx(ix) + (unit(rng) - 0.5f) * VOXEL_SIZE;
                    const float py = cy(iy) + (unit(rng) - 0.5f) * VOXEL_SIZE;
                    const float pz = cz(iz) + (unit(rng) - 0.5f) * VOXEL_SIZE;
                    if (u < 0.35f) {
                        set_plane(ix, iy, iz, px, py, pz, nx, ny, nz, 0);   // present, invalid
                    } else if (u < 0.38f) {
                        set_plane(ix, iy, iz, px, py, pz, 0, 0, 0, 1);      // zero normal
                    } else {
                        set_plane(ix, iy, iz, px, py, pz, nx, ny, nz, 1);
                    }
                }
            }
        }

        // Pose: small rotation, small translation.
        const float wx = 0.02f * gauss(rng), wy = 0.02f * gauss(rng), wz = 0.02f * gauss(rng);
        const float th = std::sqrt(wx * wx + wy * wy + wz * wz) + 1e-12f;
        const float s = std::sin(th) / th, c = (1 - std::cos(th)) / (th * th);
        const float K[9] = {0, -wz, wy, wz, 0, -wx, -wy, wx, 0};
        float R[9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                float kk = 0;
                for (int m = 0; m < 3; ++m) kk += K[3 * i + m] * K[3 * m + j];
                R[3 * i + j] = (i == j ? 1.0f : 0.0f) + s * K[3 * i + j] + c * kk;
            }
        }
        const float t[3] = {0.5f * gauss(rng), 0.5f * gauss(rng), 0.2f * gauss(rng)};

        // Scan: uniform in a box a little larger than the grid, so some points
        // fall outside and every voxel layer is exercised.
        for (uint32_t i = 0; i < points_per_run; ++i) {
            set_point(i, -9.0f + 18.0f * unit(rng), -9.0f + 18.0f * unit(rng),
                      -3.0f + 6.0f * unit(rng));
        }

        Raw r;
        const uint64_t cycles = compare("random", points_per_run, R, t, &r);
        std::printf("  random run %d: %u points, %u inliers, %u rejected, %.1f cycles/point\n", run,
                    points_per_run, r.inlier_count, r.rejected_count,
                    (double)cycles / points_per_run);
    }
}

// ---- Test vectors --------------------------------------------------------------------

std::string base_name(const std::string &path) {
    std::string p = path;
    while (!p.empty() && p[p.size() - 1] == '/') p.erase(p.size() - 1);
    const size_t slash = p.find_last_of('/');
    return slash == std::string::npos ? p : p.substr(slash + 1);
}

void run_case(const std::string &dir, const std::string &out_dir) {
    const std::string name = base_name(dir);
    std::map<std::string, double> meta;
    std::vector<float> scan, desc, pose;
    std::vector<uint8_t> valid;
    if (!tb::read_meta(dir + "/meta.txt", meta)) {
        std::printf("case %s: no meta.txt\n", name.c_str());
        CHECK(false);
        return;
    }
    const uint32_t n = (uint32_t)meta["num_points"];
    const bool loaded = n > 0 && n <= MAX_POINTS &&
                        tb::read_binary(dir + "/scan.bin", scan, (size_t)n * 3) &&
                        tb::read_binary(dir + "/voxel_desc.bin", desc, (size_t)NUM_VOXELS * 6) &&
                        tb::read_binary(dir + "/voxel_valid.bin", valid, (size_t)NUM_VOXELS) &&
                        tb::read_binary(dir + "/pose.bin", pose, 12);
    CHECK(loaded);
    if (!loaded) return;

    for (uint32_t i = 0; i < n; ++i) set_point(i, scan[3 * i], scan[3 * i + 1], scan[3 * i + 2]);
    for (int k = 0; k < NUM_VOXELS; ++k) {
        const float *d = desc.data() + 6 * k;
        g_map[k] = tb::voxel(d[0], d[1], d[2], d[3], d[4], d[5], valid[k]);
    }

    Raw r;
    const uint64_t cycles = compare(name.c_str(), n, pose.data(), pose.data() + 9, &r);
    std::printf("case %s: %u points, %u inliers, %llu cycles, %.1f cycles/point, RTL == C++ fixed: %s\n",
                name.c_str(), n, r.inlier_count, (unsigned long long)cycles, (double)cycles / n,
                tb::failures == 0 ? "yes" : "NO");
    CHECK(tb::write_result(out_dir + "/" + name + "/rtl_fixed.txt", to_result_d(r)));
}

} // namespace

int main(int argc, char **argv) {
    Rtl rtl(argc, argv);
    g_rtl = &rtl;

    directed();
    std::printf("directed scenarios: %d checks, %d failures\n", tb::checks, tb::failures);

    random_scenarios(8, 2000);

    for (int i = 2; i < argc; ++i) {
        run_case(argv[i], argv[1]);
    }

    const bool rtl_errors = rtl.ctx->gotError();
    if (rtl_errors) {
        std::printf("RTL reported $error\n");
    }
    const int rc = tb::finish("tb_rtl");
    return rc != 0 || rtl_errors ? 1 : 0;
}
