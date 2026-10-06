#ifndef VOXLIO_TB_UTIL_HPP
#define VOXLIO_TB_UTIL_HPP

// Shared helpers for the testbenches: checks, host-side type conversion,
// result files and test-vector loading. Not synthesizable; testbench only.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include "voxlio_types.hpp"

namespace tb {

static int checks = 0;
static int failures = 0;

inline void check(bool ok, const char *expr, const char *file, int line) {
    ++checks;
    if (!ok) {
        ++failures;
        std::printf("  FAIL %s:%d: %s\n", file, line, expr);
    }
}

inline void check_near(double a, double b, double tol, const char *expr,
                       const char *file, int line) {
    ++checks;
    if (!(std::fabs(a - b) <= tol)) {
        ++failures;
        std::printf("  FAIL %s:%d: %s (%.9g vs %.9g, tol %.3g)\n", file, line, expr, a, b, tol);
    }
}

#define CHECK(cond) tb::check((cond), #cond, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) \
    tb::check_near((double)(a), (double)(b), (tol), #a " ~ " #b, __FILE__, __LINE__)

inline int finish(const char *name) {
    std::printf("%s: %d checks, %d failures -> %s\n", name, checks, failures,
                failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}

// ---- Host-side conversion into the interface types -------------------------
// Float build: identity. Fixed build: round to nearest and saturate, which is
// what the host driver is expected to do. EXACT is the tolerance for values
// that the float build reproduces exactly.
#ifdef VOXLIO_FIXED_POINT
const bool FIXED_POINT = true;
const double EXACT = 2e-3;
inline coord_t to_coord(double v) {
    ap_fixed<VOXLIO_COORD_W, VOXLIO_COORD_I, AP_RND, AP_SAT> q = v;
    return q;
}
inline normal_t to_normal(double v) {
    ap_fixed<VOXLIO_NORMAL_W, VOXLIO_NORMAL_I, AP_RND, AP_SAT> q = v;
    return q;
}
#else
const bool FIXED_POINT = false;
const double EXACT = 0.0;
inline coord_t to_coord(float v) { return v; }
inline normal_t to_normal(float v) { return v; }
#endif

inline Point3D point(float x, float y, float z) {
    Point3D p;
    p.x = to_coord(x);
    p.y = to_coord(y);
    p.z = to_coord(z);
    return p;
}

inline Pose3D pose(const float R[9], const float t[3]) {
    Pose3D out;
    for (int i = 0; i < 9; ++i) {
        out.R[i] = to_normal(R[i]);
    }
    for (int i = 0; i < 3; ++i) {
        out.t[i] = to_coord(t[i]);
    }
    return out;
}

const float IDENTITY_R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
const float ZERO_T[3] = {0, 0, 0};

inline VoxelEntry voxel(float cx, float cy, float cz, float nx, float ny, float nz,
                        uint8_t valid = 1) {
    VoxelEntry v;
    v.cx = to_coord(cx);
    v.cy = to_coord(cy);
    v.cz = to_coord(cz);
    v.nx = to_normal(nx);
    v.ny = to_normal(ny);
    v.nz = to_normal(nz);
    v.valid = valid;
    return v;
}

// Centre of voxel (ix, iy, iz) in the map frame.
inline float voxel_centre(int i, float map_min) { return map_min + (i + 0.5f) * VOXEL_SIZE; }

// ---- Results ---------------------------------------------------------------

struct ResultD {
    double H[21];
    double g[6];
    double cost;
    uint32_t inlier_count;
    uint32_t processed_count;
    uint32_t rejected_count;
    uint32_t status;
};

// Works for VoxLIOResult and RefResult, which share member names.
template <typename R>
inline ResultD to_result_d(const R &r) {
    ResultD d;
    for (int k = 0; k < 21; ++k) {
        d.H[k] = (double)r.H[k];
    }
    for (int i = 0; i < 6; ++i) {
        d.g[i] = (double)r.g[i];
    }
    d.cost = (double)r.cost;
    d.inlier_count = r.inlier_count;
    d.processed_count = r.processed_count;
    d.rejected_count = r.rejected_count;
    d.status = r.status;
    return d;
}

inline bool same_counts(const ResultD &a, const ResultD &b) {
    return a.inlier_count == b.inlier_count && a.processed_count == b.processed_count &&
           a.rejected_count == b.rejected_count && a.status == b.status;
}

// Largest absolute difference over H, g and cost. NaN-safe: any NaN gives inf.
inline double max_abs_diff(const ResultD &a, const ResultD &b) {
    double worst = 0.0;
    for (int k = 0; k < 28; ++k) {
        const double x = k < 21 ? a.H[k] : (k < 27 ? a.g[k - 21] : a.cost);
        const double y = k < 21 ? b.H[k] : (k < 27 ? b.g[k - 21] : b.cost);
        const double d = std::fabs(x - y);
        if (!(d <= worst)) {
            worst = (d == d) ? d : HUGE_VAL;
        }
    }
    return worst;
}

inline double max_abs_h(const ResultD &a) {
    double worst = 0.0;
    for (int k = 0; k < 21; ++k) {
        worst = std::fmax(worst, std::fabs(a.H[k]));
    }
    return worst;
}

// "key [index] value" lines; floats as C99 hex so they round-trip exactly
// (same format as reference/voxlio_reference.py).
inline bool write_result(const std::string &path, const ResultD &r) {
    std::FILE *f = std::fopen(path.c_str(), "w");
    if (!f) {
        return false;
    }
    std::fprintf(f, "inlier_count %u\nprocessed_count %u\nrejected_count %u\nstatus %u\n",
                 r.inlier_count, r.processed_count, r.rejected_count, r.status);
    for (int k = 0; k < 21; ++k) {
        std::fprintf(f, "H %d %a\n", k, r.H[k]);
    }
    for (int i = 0; i < 6; ++i) {
        std::fprintf(f, "g %d %a\n", i, r.g[i]);
    }
    std::fprintf(f, "cost %a\n", r.cost);
    return std::fclose(f) == 0;
}

inline bool read_result(const std::string &path, ResultD &r) {
    std::FILE *f = std::fopen(path.c_str(), "r");
    if (!f) {
        return false;
    }
    r = ResultD();
    char key[32], a[64], b[64];
    char line[256];
    int seen = 0;
    while (std::fgets(line, sizeof line, f)) {
        const int n = std::sscanf(line, "%31s %63s %63s", key, a, b);
        const std::string k(n >= 1 ? key : "");
        if (k == "H" && n == 3) {
            r.H[std::atoi(a)] = std::strtod(b, 0);
        } else if (k == "g" && n == 3) {
            r.g[std::atoi(a)] = std::strtod(b, 0);
        } else if (k == "cost" && n == 2) {
            r.cost = std::strtod(a, 0);
        } else if (k == "inlier_count" && n == 2) {
            r.inlier_count = (uint32_t)std::strtoul(a, 0, 10);
        } else if (k == "processed_count" && n == 2) {
            r.processed_count = (uint32_t)std::strtoul(a, 0, 10);
        } else if (k == "rejected_count" && n == 2) {
            r.rejected_count = (uint32_t)std::strtoul(a, 0, 10);
        } else if (k == "status" && n == 2) {
            r.status = (uint32_t)std::strtoul(a, 0, 10);
        } else {
            continue;
        }
        ++seen;
    }
    std::fclose(f);
    return seen == 32;
}

// ---- Test vectors ----------------------------------------------------------

template <typename T>
inline bool read_binary(const std::string &path, std::vector<T> &out, size_t count) {
    std::FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) {
        return false;
    }
    out.resize(count);
    const size_t got = std::fread(out.data(), sizeof(T), count, f);
    const bool at_end = std::fgetc(f) == EOF;
    std::fclose(f);
    return got == count && at_end;
}

// meta.txt: "key value" lines; non-numeric values are skipped.
inline bool read_meta(const std::string &path, std::map<std::string, double> &meta) {
    std::FILE *f = std::fopen(path.c_str(), "r");
    if (!f) {
        return false;
    }
    char key[64], value[64];
    char line[512];
    while (std::fgets(line, sizeof line, f)) {
        if (std::sscanf(line, "%63s %63s", key, value) == 2) {
            char *end = 0;
            const double v = std::strtod(value, &end);
            if (end != value && *end == '\0') {
                meta[key] = v;
            }
        }
    }
    std::fclose(f);
    return true;
}

} // namespace tb

#endif // VOXLIO_TB_UTIL_HPP
