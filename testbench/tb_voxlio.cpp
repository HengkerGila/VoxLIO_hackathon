// Top-level testbench for voxlio_core.
//
// Part 1: directed tests (acceptance tests 1-5 and the defined edge
//         cases) on small hand-built maps.
// Part 2: test vectors exported by scripts/export_vectors.py. In the float
//         build: Python == plain C++ (test 8) and plain C++ == HLS core
//         (test 9). In the fixed-point build: fixed core vs float reference
//         within the selected tolerance (test 10).
//
// Usage: tb_voxlio [output_dir case_dir...]
// Every run also drives the plain C++ reference with the same inputs.

#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "tb_util.hpp"
#include "voxlio_core.hpp"
#include "voxlio_ref.hpp"

namespace {

// Inputs as raw float arrays (what the Python model and the plain reference
// consume) and the same data in the interface types of the core under test.
float g_scan_f[MAX_POINTS * 3];
float g_desc_f[NUM_VOXELS * 6];
uint8_t g_valid[NUM_VOXELS];
Point3D g_scan[MAX_POINTS];
VoxelEntry g_map[NUM_VOXELS];

const uint32_t OK = VOXLIO_STATUS_SUCCESS;
const uint32_t ZERO = VOXLIO_STATUS_ZERO_INLIERS;
const uint32_t OOB = VOXLIO_STATUS_OOB_PREVENTED;

int flat(int ix, int iy, int iz) { return (iz * NY + iy) * NX + ix; }
float cx(int ix) { return tb::voxel_centre(ix, MAP_X_MIN); }
float cy(int iy) { return tb::voxel_centre(iy, MAP_Y_MIN); }
float cz(int iz) { return tb::voxel_centre(iz, MAP_Z_MIN); }

void clear_map() {
    std::memset(g_desc_f, 0, sizeof g_desc_f);
    std::memset(g_valid, 0, sizeof g_valid);
}

void set_plane(int ix, int iy, int iz, float px, float py, float pz, float nx, float ny,
               float nz, uint8_t valid = 1) {
    float *d = g_desc_f + 6 * flat(ix, iy, iz);
    d[0] = px;
    d[1] = py;
    d[2] = pz;
    d[3] = nx;
    d[4] = ny;
    d[5] = nz;
    g_valid[flat(ix, iy, iz)] = valid;
}

// One valid voxel: the horizontal plane at height z in the voxel at the
// map-frame origin, (16, 16, 4).
void ground_map(float z = 0.0f) {
    clear_map();
    set_plane(16, 16, 4, 0, 0, z, 0, 0, 1);
}

void set_point(uint32_t i, float x, float y, float z) {
    g_scan_f[3 * i + 0] = x;
    g_scan_f[3 * i + 1] = y;
    g_scan_f[3 * i + 2] = z;
}

struct Run {
    tb::ResultD hls;
    tb::ResultD ref;
};

Run run(uint32_t num_points, const float R[9] = tb::IDENTITY_R,
        const float t[3] = tb::ZERO_T) {
    const uint32_t stored = num_points <= MAX_POINTS ? num_points : MAX_POINTS;
    for (uint32_t i = 0; i < stored; ++i) {
        g_scan[i] = tb::point(g_scan_f[3 * i], g_scan_f[3 * i + 1], g_scan_f[3 * i + 2]);
    }
    for (int k = 0; k < NUM_VOXELS; ++k) {
        const float *d = g_desc_f + 6 * k;
        g_map[k] = tb::voxel(d[0], d[1], d[2], d[3], d[4], d[5], g_valid[k]);
    }

    VoxLIOResult out;
    voxlio_core(g_scan, num_points, g_map, tb::pose(R, t), out);

    Run r;
    r.hls = tb::to_result_d(out);
    r.ref = tb::to_result_d(voxlio_ref(g_scan_f, num_points, g_desc_f, g_valid, R, t));
#ifndef VOXLIO_FIXED_POINT
    // The float core and the plain reference are the same arithmetic.
    CHECK(tb::same_counts(r.hls, r.ref));
    CHECK(tb::max_abs_diff(r.hls, r.ref) == 0.0);
#endif
    return r;
}

void expect_empty(const tb::ResultD &r, uint32_t status, uint32_t processed,
                  uint32_t rejected) {
    CHECK(r.inlier_count == 0);
    CHECK(r.processed_count == processed);
    CHECK(r.rejected_count == rejected);
    CHECK(r.status == status);
    bool zero = r.cost == 0.0;
    for (int k = 0; k < 21; ++k) {
        zero = zero && r.H[k] == 0.0;
    }
    for (int i = 0; i < 6; ++i) {
        zero = zero && r.g[i] == 0.0;
    }
    CHECK(zero);
}

// H, g and cost of a single constraint, evaluated in float like the core.
void expect_constraints(const tb::ResultD &r, const float (*J)[6], const float *res,
                        int count) {
    float H[21] = {0}, g[6] = {0}, cost = 0;
    for (int c = 0; c < count; ++c) {
        for (int j = 0; j < 6; ++j) {
            for (int i = 0; i <= j; ++i) {
                H[i + j * (j + 1) / 2] += J[c][i] * J[c][j];
            }
        }
        for (int i = 0; i < 6; ++i) {
            g[i] += J[c][i] * res[c];
        }
        cost += res[c] * res[c];
    }
    for (int k = 0; k < 21; ++k) {
        CHECK_NEAR(r.H[k], H[k], tb::EXACT);
    }
    for (int i = 0; i < 6; ++i) {
        CHECK_NEAR(r.g[i], g[i], tb::EXACT);
    }
    CHECK_NEAR(r.cost, cost, tb::EXACT);
}

// ---- Part 1: directed tests ------------------------------------------------

void test_1_single_point_known_residual() {
    ground_map();
    set_point(0, 0.1f, -0.05f, 0.125f);
    const Run r = run(1);
    CHECK(r.hls.inlier_count == 1 && r.hls.rejected_count == 0 && r.hls.status == OK);
    // J = [py*nz - pz*ny, pz*nx - px*nz, px*ny - py*nx, n] with n = +z.
    const float J[1][6] = {{-0.05f, -0.1f, 0, 0, 0, 1}};
    const float res[1] = {0.125f};
    expect_constraints(r.hls, J, res, 1);
    CHECK_NEAR(r.hls.cost, 0.015625, tb::EXACT);
}

void test_2_point_on_plane() {
    ground_map();
    set_point(0, 0.2f, 0.1f, 0.0f);
    const Run r = run(1);
    CHECK(r.hls.inlier_count == 1 && r.hls.status == OK);
    CHECK_NEAR(r.hls.cost, 0.0, tb::EXACT);
    for (int i = 0; i < 6; ++i) {
        CHECK_NEAR(r.hls.g[i], 0.0, tb::EXACT);
    }
    CHECK_NEAR(r.hls.H[20], 1.0, tb::EXACT); // H55 = nz^2
}

void test_3_point_outside_map() {
    ground_map();
    set_point(0, 100.0f, 0.0f, 0.0f);
    expect_empty(run(1).hls, ZERO | OOB, 1, 1);
}

void test_4_boundary_voxels() {
    // A plane in a face/corner voxel is matched like any other.
    const int corners[4][3] = {
        {0, 0, 0}, {NX - 1, NY - 1, NZ - 1}, {0, NY - 1, 3}, {NX - 1, 0, 0}};
    for (int c = 0; c < 4; ++c) {
        const int *v = corners[c];
        clear_map();
        set_plane(v[0], v[1], v[2], cx(v[0]), cy(v[1]), cz(v[2]), 0, 0, 1);
        set_point(0, cx(v[0]), cy(v[1]), cz(v[2]) + 0.0625f);
        const Run r = run(1);
        CHECK(r.hls.inlier_count == 1 && r.hls.status == OK);
        CHECK_NEAR(r.hls.cost, 0.0625 * 0.0625, tb::EXACT);
    }

    // The only valid voxel is where an unchecked neighbour address would
    // land; its plane passes through the query point, so it would win if it
    // were ever read. (The sanitizer build also traps real out-of-range reads.)
    const int traps[4][2][3] = {
        {{0, 1, 0}, {NX - 1, 0, 0}},                   // x - 1 wraps to the row below
        {{NX - 1, 0, 0}, {0, 1, 0}},                   // x + 1 wraps to the next row
        {{0, 0, 0}, {NX - 1, NY - 1, NZ - 1}},         // address -1
        {{NX - 1, NY - 1, NZ - 1}, {0, 0, 0}},         // address NUM_VOXELS
    };
    for (int c = 0; c < 4; ++c) {
        const int *centre = traps[c][0];
        const int *trap = traps[c][1];
        clear_map();
        set_plane(trap[0], trap[1], trap[2], cx(centre[0]), cy(centre[1]), cz(centre[2]),
                  0, 0, 1);
        set_point(0, cx(centre[0]), cy(centre[1]), cz(centre[2]));
        expect_empty(run(1).hls, ZERO, 1, 1);
    }
}

void test_5_all_neighbours_invalid() {
    ground_map();
    std::memset(g_valid, 0, sizeof g_valid); // descriptors present, none valid
    set_point(0, 0.0f, 0.0f, 0.05f);
    expect_empty(run(1).hls, ZERO, 1, 1);
}

void test_two_constraints() {
    ground_map();
    set_point(0, 0.1f, -0.05f, 0.125f);
    set_point(1, -0.2f, 0.15f, -0.0625f);
    const Run r = run(2);
    CHECK(r.hls.inlier_count == 2 && r.hls.status == OK);
    const float J[2][6] = {{-0.05f, -0.1f, 0, 0, 0, 1}, {0.15f, 0.2f, 0, 0, 0, 1}};
    const float res[2] = {0.125f, -0.0625f};
    expect_constraints(r.hls, J, res, 2);
    CHECK_NEAR(r.hls.H[20], 2.0, tb::EXACT);
}

void test_plane_selection() {
    const double tol = tb::FIXED_POINT ? 2e-3 : 1e-7;

    // The smaller |residual| wins regardless of visiting order.
    ground_map(0.20f);
    set_plane(17, 16, 4, 0.5f, 0.0f, -0.05f, 0, 0, 1);
    set_point(0, 0, 0, 0);
    CHECK_NEAR(run(1).hls.g[5], 0.05, tol);

    // Equal |residual|: the first candidate in dz, dy, dx order is kept.
    // (16,16,4) is visited before (17,16,4).
    ground_map(0.125f);
    set_plane(17, 16, 4, 0.5f, 0.0f, -0.125f, 0, 0, 1);
    Run r = run(1);
    CHECK(r.hls.inlier_count == 1);
    CHECK_NEAR(r.hls.g[5], -0.125, tb::EXACT);

    // The search reaches exactly one voxel in every direction.
    const int cases[4][4] = {{17, 17, 5, 1}, {15, 15, 3, 1}, {18, 16, 4, 0}, {16, 16, 6, 0}};
    for (int c = 0; c < 4; ++c) {
        clear_map();
        set_plane(cases[c][0], cases[c][1], cases[c][2], 0, 0, 0, 0, 0, 1);
        CHECK(run(1).hls.inlier_count == (uint32_t)cases[c][3]);
    }
}

void test_inlier_threshold() {
    set_point(0, 0, 0, 0);
    ground_map(-0.25f);
    CHECK(run(1).hls.inlier_count == 1);
    ground_map(-0.35f);
    Run r = run(1);
    CHECK(r.hls.inlier_count == 0 && r.hls.rejected_count == 1 && r.hls.status == ZERO);
#ifndef VOXLIO_FIXED_POINT
    // Strict comparison: |r| == threshold is rejected, one float below is kept.
    const float threshold = RESIDUAL_THRESHOLD;
    ground_map(-threshold);
    CHECK(run(1).hls.inlier_count == 0);
    ground_map(-std::nextafter(threshold, 0.0f));
    CHECK(run(1).hls.inlier_count == 1);
#endif
}

void test_point_count_limits() {
    ground_map();

    expect_empty(run(0).hls, ZERO, 0, 0);

    // Too many points: the run is aborted before any point is read.
    set_point(0, 0.0f, 0.0f, 0.125f);
    expect_empty(run(MAX_POINTS + 1).hls, ZERO | VOXLIO_STATUS_POINT_OVERFLOW, 0, 0);
    expect_empty(run(0xFFFFFFFFu).hls, ZERO | VOXLIO_STATUS_POINT_OVERFLOW, 0, 0);

    // Exactly MAX_POINTS is legal; every point is visited.
    for (uint32_t i = 0; i < MAX_POINTS; ++i) {
        set_point(i, 100.0f, 0.0f, 0.0f);
    }
    set_point(MAX_POINTS - 1, 0.0f, 0.0f, 0.125f);
    const Run r = run(MAX_POINTS);
    CHECK(r.hls.processed_count == MAX_POINTS && r.hls.inlier_count == 1);
    CHECK(r.hls.rejected_count == MAX_POINTS - 1 && r.hls.status == (OK | OOB));
}

void test_empty_and_outside() {
    // All points outside the map.
    ground_map();
    set_point(0, 50, 0, 0);
    set_point(1, 0, -50, 0);
    set_point(2, 0, 0, 9);
    expect_empty(run(3).hls, ZERO | OOB, 3, 3);

    // Empty map.
    clear_map();
    set_point(0, 0, 0, 0);
    set_point(1, 1, 1, 0);
    expect_empty(run(2).hls, ZERO, 2, 2);
}

void test_zero_length_normal() {
    clear_map();
    set_plane(16, 16, 4, 0, 0, 0, 0, 0, 0);
    set_point(0, 0.0f, 0.0f, 0.0625f);
    expect_empty(run(1).hls, ZERO, 1, 1);

    // It must not shadow a real plane next to it (its residual would be 0).
    set_plane(17, 16, 4, 0.5f, 0, 0, 0, 0, 1);
    const Run r = run(1);
    CHECK(r.hls.inlier_count == 1);
    CHECK_NEAR(r.hls.g[5], 0.0625, tb::EXACT);
}

void test_extreme_pose() {
    ground_map();
    set_point(0, 0, 0, 0);
    set_point(1, 1, 1, 0);
    const float far_t[3] = {1e30f, 0, 0};
    expect_empty(run(2, tb::IDENTITY_R, far_t).hls, ZERO | OOB, 2, 2);

    // 511 + 511 = 1022 m: a wrapping 10-bit-integer coordinate would land at
    // -2 m, inside the map, on a plane placed there. It must stay outside.
    clear_map();
    set_plane(12, 16, 4, -2.0f, 0, 0, 0, 0, 1);
    set_point(0, 511.0f, 0, 0);
    const float t[3] = {511.0f, 0, 0};
    expect_empty(run(1, tb::IDENTITY_R, t).hls, ZERO | OOB, 1, 1);
}

#ifndef VOXLIO_FIXED_POINT
// NaN and Inf only exist on the float interface.
void test_non_finite_inputs() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float bad[3] = {nan, inf, -inf};

    for (int b = 0; b < 3; ++b) {
        // Non-finite points are rejected and do not poison the accumulators.
        ground_map();
        set_point(0, bad[b], 0, 0);
        set_point(1, 0, bad[b], 0);
        set_point(2, 0, 0, bad[b]);
        set_point(3, 0.0f, 0.0f, 0.125f);
        const Run r = run(4);
        CHECK(r.hls.inlier_count == 1 && r.hls.rejected_count == 3);
        CHECK(r.hls.status == (OK | OOB));
        CHECK(r.hls.cost == 0.015625);
        bool finite = true;
        for (int k = 0; k < 21; ++k) {
            finite = finite && std::isfinite(r.hls.H[k]);
        }
        CHECK(finite);

        // A non-finite pose aborts the run.
        for (int slot = 0; slot < 12; ++slot) {
            float R[9], t[3];
            std::memcpy(R, tb::IDENTITY_R, sizeof R);
            std::memcpy(t, tb::ZERO_T, sizeof t);
            (slot < 9 ? R[slot] : t[slot - 9]) = bad[b];
            expect_empty(run(4, R, t).hls, ZERO | VOXLIO_STATUS_BAD_POSE, 0, 0);
        }

        // A non-finite descriptor never wins, alone or next to a real plane.
        for (int field = 0; field < 6; ++field) {
            ground_map();
            g_desc_f[6 * flat(16, 16, 4) + field] = bad[b];
            set_point(0, 0.0f, 0.0f, 0.0625f);
            expect_empty(run(1).hls, ZERO, 1, 1);
            set_plane(17, 16, 4, 0.5f, 0, 0, 0, 0, 1);
            const Run both = run(1);
            CHECK(both.hls.inlier_count == 1 && both.hls.g[5] == 0.0625);
        }
    }
}
#endif

// ---- Part 2: exported test vectors -----------------------------------------

std::string base_name(const std::string &path) {
    std::string p = path;
    while (!p.empty() && p[p.size() - 1] == '/') {
        p.erase(p.size() - 1);
    }
    const size_t slash = p.find_last_of('/');
    return slash == std::string::npos ? p : p.substr(slash + 1);
}

void run_case(const std::string &dir, const std::string &out_dir) {
    const std::string name = base_name(dir);
    std::printf("case %s\n", name.c_str());

    std::map<std::string, double> meta;
    CHECK(tb::read_meta(dir + "/meta.txt", meta));
    // The vectors must have been generated for this build's grid.
    const bool same_grid = meta["nx"] == NX && meta["ny"] == NY && meta["nz"] == NZ &&
                           meta["voxel_size"] == (double)VOXEL_SIZE &&
                           meta["map_x_min"] == (double)MAP_X_MIN &&
                           meta["map_y_min"] == (double)MAP_Y_MIN &&
                           meta["map_z_min"] == (double)MAP_Z_MIN &&
                           meta["residual_threshold"] == (double)RESIDUAL_THRESHOLD &&
                           meta["neighbor_radius"] == NEIGHBOR_RADIUS;
    CHECK(same_grid);
    const uint32_t num_points = (uint32_t)meta["num_points"];
    CHECK(num_points > 0 && num_points <= MAX_POINTS);
    if (!same_grid || num_points == 0 || num_points > MAX_POINTS) {
        std::printf("  stale or unreadable vectors in %s; rerun scripts/export_vectors.py\n",
                    dir.c_str());
        return;
    }

    std::vector<float> scan, desc, pose;
    std::vector<uint8_t> valid;
    const bool loaded = tb::read_binary(dir + "/scan.bin", scan, (size_t)num_points * 3) &&
                        tb::read_binary(dir + "/voxel_desc.bin", desc, (size_t)NUM_VOXELS * 6) &&
                        tb::read_binary(dir + "/voxel_valid.bin", valid, (size_t)NUM_VOXELS) &&
                        tb::read_binary(dir + "/pose.bin", pose, 12);
    tb::ResultD python;
    const bool have_expected = tb::read_result(dir + "/expected_f32.txt", python);
    CHECK(loaded);
    CHECK(have_expected);
    if (!loaded || !have_expected) {
        return;
    }
    std::memcpy(g_scan_f, scan.data(), scan.size() * sizeof(float));
    std::memcpy(g_desc_f, desc.data(), desc.size() * sizeof(float));
    std::memcpy(g_valid, valid.data(), valid.size());

    const Run r = run(num_points, pose.data(), pose.data() + 9);
    const double scale = tb::max_abs_h(python);

    // Test 8: Python golden model == plain C++ reference.
    const double py_diff = tb::max_abs_diff(r.ref, python);
    std::printf("  python f32 vs C++ ref : counts %s, max |diff| %.3g (%s)\n",
                tb::same_counts(r.ref, python) ? "equal" : "DIFFER", py_diff,
                py_diff == 0.0 ? "bit-exact" : "not bit-exact");
    CHECK(tb::same_counts(r.ref, python));
    CHECK(py_diff <= 1e-6 * scale);
    CHECK(tb::write_result(out_dir + "/" + name + "/cpp_ref.txt", r.ref));

    const double hls_diff = tb::max_abs_diff(r.hls, r.ref);
    const long inlier_delta = (long)r.hls.inlier_count - (long)r.ref.inlier_count;
#ifndef VOXLIO_FIXED_POINT
    // Test 9: plain C++ reference == HLS core.
    std::printf("  C++ ref vs HLS float  : counts %s, max |diff| %.3g (%s)\n",
                tb::same_counts(r.hls, r.ref) ? "equal" : "DIFFER", hls_diff,
                hls_diff == 0.0 ? "bit-exact" : "not bit-exact");
    CHECK(tb::same_counts(r.hls, r.ref));
    CHECK(hls_diff <= 1e-6 * scale);
    CHECK(tb::write_result(out_dir + "/" + name + "/hls_float.txt", r.hls));
#else
    // Test 10: fixed-point core vs float reference. Coarse gate here; the full
    // metrics and the selected tolerance are applied by compare_outputs.py.
    std::printf("  C++ ref vs HLS fixed  : inliers %+ld, max |diff| %.3g (%.3g of max |H|)\n",
                inlier_delta, hls_diff, hls_diff / scale);
    CHECK(r.hls.processed_count == r.ref.processed_count);
    CHECK(std::labs(inlier_delta) <= (long)(num_points / 200));
    CHECK(hls_diff <= 1e-2 * scale);
    CHECK(tb::write_result(out_dir + "/" + name + "/hls_fixed.txt", r.hls));
#endif
    (void)inlier_delta;
}

} // namespace

int main(int argc, char **argv) {
    test_1_single_point_known_residual();
    test_2_point_on_plane();
    test_3_point_outside_map();
    test_4_boundary_voxels();
    test_5_all_neighbours_invalid();
    test_two_constraints();
    test_plane_selection();
    test_inlier_threshold();
    test_point_count_limits();
    test_empty_and_outside();
    test_zero_length_normal();
    test_extreme_pose();
#ifndef VOXLIO_FIXED_POINT
    test_non_finite_inputs();
#endif
    std::printf("directed tests: %d checks, %d failures\n", tb::checks, tb::failures);

    if (argc < 3) {
        std::printf("no test vectors given (usage: tb_voxlio output_dir case_dir...)\n");
    }
    for (int i = 2; i < argc; ++i) {
        run_case(argv[i], argv[1]);
    }

    return tb::finish(tb::FIXED_POINT ? "tb_voxlio (fixed)" : "tb_voxlio (float)");
}
