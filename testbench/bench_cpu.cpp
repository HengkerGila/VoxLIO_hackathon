// CPU baseline: wall-clock time of the plain C++ reference on one test vector
// ("CPU Reference" column of the benchmark matrix).
//
// Usage: bench_cpu case_dir [repeats]
// Prints: points, inliers, median and minimum milliseconds per scan.

#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <vector>

#include "tb_util.hpp"
#include "voxlio_ref.hpp"

int main(int argc, char **argv) {
    if (argc < 2) {
        std::printf("usage: bench_cpu case_dir [repeats]\n");
        return 2;
    }
    const std::string dir = argv[1];
    const int repeats = argc > 2 ? std::atoi(argv[2]) : 200;

    std::map<std::string, double> meta;
    std::vector<float> scan, desc, pose;
    std::vector<uint8_t> valid;
    if (!tb::read_meta(dir + "/meta.txt", meta)) {
        std::printf("cannot read %s/meta.txt\n", dir.c_str());
        return 2;
    }
    const uint32_t num_points = (uint32_t)meta["num_points"];
    if (!tb::read_binary(dir + "/scan.bin", scan, (size_t)num_points * 3) ||
        !tb::read_binary(dir + "/voxel_desc.bin", desc, (size_t)NUM_VOXELS * 6) ||
        !tb::read_binary(dir + "/voxel_valid.bin", valid, (size_t)NUM_VOXELS) ||
        !tb::read_binary(dir + "/pose.bin", pose, 12)) {
        std::printf("cannot read vectors in %s\n", dir.c_str());
        return 2;
    }

    std::vector<double> ms;
    uint32_t inliers = 0;
    float checksum = 0.0f;
    for (int r = 0; r < repeats; ++r) {
        const auto start = std::chrono::steady_clock::now();
        const RefResult res = voxlio_ref(scan.data(), num_points, desc.data(), valid.data(),
                                         pose.data(), pose.data() + 9);
        const auto stop = std::chrono::steady_clock::now();
        ms.push_back(std::chrono::duration<double, std::milli>(stop - start).count());
        inliers = res.inlier_count;
        checksum += res.cost; // keeps the call from being optimised away
    }
    std::sort(ms.begin(), ms.end());
    std::printf("points %u inliers %u repeats %d median_ms %.4f min_ms %.4f checksum %.3f\n",
                num_points, inliers, repeats, ms[ms.size() / 2], ms[0], checksum);
    return 0;
}
