// Unit test: stages 2-3 (bounds check, voxel index) and flattened addressing.

#include <limits>

#include "tb_util.hpp"
#include "voxel_index.hpp"
#include "voxel_lookup.hpp"

static bool index_of(float x, float y, float z, int &ix, int &iy, int &iz) {
    return voxel_index(tb::point(x, y, z), ix, iy, iz);
}

int main() {
    int ix = -1, iy = -1, iz = -1;
    const float x_max = MAP_X_MIN + NX * VOXEL_SIZE;
    const float y_max = MAP_Y_MIN + NY * VOXEL_SIZE;
    const float z_max = MAP_Z_MIN + NZ * VOXEL_SIZE;

    // The map-frame origin is in the centre voxel.
    CHECK(index_of(0, 0, 0, ix, iy, iz));
    CHECK(ix == 16 && iy == 16 && iz == 4);

    // Negative world coordinates; the minimum corner is inclusive.
    CHECK(index_of(MAP_X_MIN, MAP_Y_MIN, MAP_Z_MIN, ix, iy, iz));
    CHECK(ix == 0 && iy == 0 && iz == 0);
    CHECK(index_of(-8.0f, -7.6f, -1.9f, ix, iy, iz));
    CHECK(ix == 0 && iy == 1 && iz == 0);

    // The maximum corner is exclusive; just inside it is the last voxel.
    CHECK(!index_of(x_max, 0, 0, ix, iy, iz));
    CHECK(!index_of(0, y_max, 0, ix, iy, iz));
    CHECK(!index_of(0, 0, z_max, ix, iy, iz));
    CHECK(index_of(x_max - 0.001f, y_max - 0.001f, z_max - 0.001f, ix, iy, iz));
    CHECK(ix == NX - 1 && iy == NY - 1 && iz == NZ - 1);

    // Outside on each side of each axis.
    CHECK(!index_of(MAP_X_MIN - 0.001f, 0, 0, ix, iy, iz));
    CHECK(!index_of(0, MAP_Y_MIN - 0.001f, 0, ix, iy, iz));
    CHECK(!index_of(0, 0, MAP_Z_MIN - 0.001f, ix, iy, iz));
    CHECK(!index_of(100.0f, 0, 0, ix, iy, iz));
    CHECK(!index_of(0, -100.0f, 0, ix, iy, iz));
    CHECK(!index_of(0, 0, 50.0f, ix, iy, iz));
    // Far beyond any coordinate format: must saturate/compare, never wrap.
    CHECK(!index_of(1e30f, 0, 0, ix, iy, iz));
    CHECK(!index_of(0, -1e30f, 0, ix, iy, iz));

#ifndef VOXLIO_FIXED_POINT
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float bad[3] = {nan, inf, -inf};
    for (int b = 0; b < 3; ++b) {
        CHECK(!index_of(bad[b], 0, 0, ix, iy, iz));
        CHECK(!index_of(0, bad[b], 0, ix, iy, iz));
        CHECK(!index_of(0, 0, bad[b], ix, iy, iz));
    }

    // Walk the floats on both sides of every face: an accepted point always
    // has in-range indices, even where (v - min) * 2 rounds.
    const float faces[6] = {MAP_X_MIN, x_max, MAP_Y_MIN, y_max, MAP_Z_MIN, z_max};
    for (int f = 0; f < 6; ++f) {
        float below = faces[f], above = faces[f];
        for (int step = 0; step < 64; ++step) {
            const float v[2] = {below, above};
            for (int s = 0; s < 2; ++s) {
                const float xyz[3] = {f / 2 == 0 ? v[s] : 0.0f, f / 2 == 1 ? v[s] : 0.0f,
                                      f / 2 == 2 ? v[s] : 0.0f};
                if (index_of(xyz[0], xyz[1], xyz[2], ix, iy, iz)) {
                    CHECK(ix >= 0 && ix < NX && iy >= 0 && iy < NY && iz >= 0 && iz < NZ);
                }
            }
            below = std::nextafter(below, -1e9f);
            above = std::nextafter(above, 1e9f);
        }
    }
#endif

    // Flattened address: index = iz * NX * NY + iy * NX + ix.
    CHECK(flatten_index(0, 0, 0) == 0);
    CHECK(flatten_index(1, 0, 0) == 1);
    CHECK(flatten_index(0, 1, 0) == (uint32_t)NX);
    CHECK(flatten_index(0, 0, 1) == (uint32_t)(NX * NY));
    CHECK(flatten_index(NX - 1, NY - 1, NZ - 1) == (uint32_t)(NUM_VOXELS - 1));

    // Every voxel gets a distinct in-range address.
    static bool seen[NUM_VOXELS];
    bool bijection = true;
    for (int z = 0; z < NZ; ++z) {
        for (int y = 0; y < NY; ++y) {
            for (int x = 0; x < NX; ++x) {
                const uint32_t a = flatten_index(x, y, z);
                if (a >= (uint32_t)NUM_VOXELS || seen[a]) {
                    bijection = false;
                } else {
                    seen[a] = true;
                }
            }
        }
    }
    CHECK(bijection);

    return tb::finish("tb_voxel_index");
}
