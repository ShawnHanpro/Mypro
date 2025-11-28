// src/map_parser_node.cpp
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring> 

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

using Byte = uint8_t;

static inline uint32_t read_u32_le(const std::vector<Byte>& b, size_t off) {
    if (off + 4 > b.size()) return 0;
    return uint32_t(b[off]) | (uint32_t(b[off + 1]) << 8) | (uint32_t(b[off + 2]) << 16) | (uint32_t(b[off + 3]) << 24);
}
static inline int32_t read_i32_le(const std::vector<Byte>& b, size_t off) { return static_cast<int32_t>(read_u32_le(b, off)); }
static inline float   read_f32_le(const std::vector<Byte>& b, size_t off) {
    if (off + 4 > b.size()) return 0.0f;
    uint32_t v = read_u32_le(b, off);
    float    f;
    std::memcpy(&f, &v, sizeof(f));
    return f;
}

struct Candidate {
    size_t  header_off;
    int32_t w;
    int32_t h;
    size_t  grid_off;
    int     extra;
};

std::vector<Candidate> find_candidates(const std::vector<Byte>& data, size_t marker_idx) {
    std::vector<Candidate> results;
    size_t                 n = data.size();
    size_t                 search_start = (marker_idx > 512) ? (marker_idx - 512) : 0;
    size_t                 search_end = std::min(n, marker_idx + 2048);

    std::vector<int> extras = {8, 12, 16, 20, 24, 28, 32};
    for (size_t off = search_start; off + 8 < search_end; ++off) {
        int32_t w = read_i32_le(data, off);
        int32_t h = read_i32_le(data, off + 4);
        if (w <= 0 || h <= 0) continue;
        if (w > 5000 || h > 5000) continue;
        // check area plausible
        if ((uint64_t)w * (uint64_t)h > (uint64_t)(n / 4)) continue;  // area too big relative to file
        for (int extra : extras) {
            size_t   start_grid = off + extra;
            uint64_t need = (uint64_t)w * (uint64_t)h;
            if (start_grid + need > n) continue;
            // check variety in first min(1000, need) bytes
            size_t           sample_len = (size_t)std::min<uint64_t>(need, 1000);
            std::vector<int> hist(256, 0);
            for (size_t i = 0; i < sample_len; ++i) hist[data[start_grid + i]]++;
            int distinct = 0;
            for (int v = 0; v < 256; ++v)
                if (hist[v]) distinct++;
            if (distinct < 3) continue;  // too uniform likely not map
            // acceptable candidate
            Candidate c;
            c.header_off = off;
            c.w = w;
            c.h = h;
            c.grid_off = start_grid;
            c.extra = extra;
            results.push_back(c);
        }
    }
    return results;
}

void write_pgm(const std::string& filename, int w, int h, const std::vector<uint8_t>& raw) {
    std::ofstream fo(filename, std::ios::binary);
    fo << "P5\n" << w << " " << h << "\n255\n";
    fo.write(reinterpret_cast<const char*>(raw.data()), raw.size());
    fo.close();
}

void write_yaml(const std::string& filename, double resolution, double origin_x, double origin_y, const std::string& pgm_name) {
    std::ofstream f(filename);
    f << "image: " << pgm_name << "\n";
    f << "resolution: " << std::fixed << std::setprecision(6) << resolution << "\n";
    f << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
    f << "negate: 0\n";
    f << "occupied_thresh: 0.65\n";
    f << "free_thresh: 0.196\n";
    f.close();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sancan_map_parser");

    if (argc < 2) {
        RCLCPP_ERROR(node->get_logger(), "Usage: ros2 run <pkg> map_parser_node <scheme_file> [--resolution RES] [--choose N]");
        return 1;
    }
    std::string fname = argv[1];

    double resolution_param = node->declare_parameter("resolution", 0.05);
    int    choose_idx = node->declare_parameter("choose", -1);

    // override with argv optional --resolution and --choose
    for (int i = 2; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--resolution" && i + 1 < argc) {
            resolution_param = atof(argv[++i]);
        } else if (a == "--choose" && i + 1 < argc) {
            choose_idx = atoi(argv[++i]);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Reading file: %s", fname.c_str());
    std::ifstream fi(fname, std::ios::binary);
    if (!fi) {
        RCLCPP_ERROR(node->get_logger(), "Cannot open file");
        return 2;
    }
    std::vector<Byte> data((std::istreambuf_iterator<char>(fi)), std::istreambuf_iterator<char>());
    fi.close();
    RCLCPP_INFO(node->get_logger(), "File size: %zu", data.size());

    std::string marker = "COccupancyGridMap2D";
    size_t      marker_idx = std::string::npos;
    for (size_t i = 0; i + marker.size() <= data.size(); ++i) {
        bool ok = true;
        for (size_t j = 0; j < marker.size(); ++j)
            if (data[i + j] != (Byte)marker[j]) {
                ok = false;
                break;
            }
        if (ok) {
            marker_idx = i;
            break;
        }
    }
    if (marker_idx == std::string::npos) {
        RCLCPP_WARN(node->get_logger(), "Marker '%s' not found, continuing search from file start", marker.c_str());
        marker_idx = 0;
    } else {
        RCLCPP_INFO(node->get_logger(), "Found marker at offset %zu", marker_idx);
    }

    auto candidates = find_candidates(data, marker_idx);
    RCLCPP_INFO(node->get_logger(), "Found %zu candidate maps", candidates.size());

    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& c = candidates[i];
        RCLCPP_INFO(node->get_logger(), "[%zu] header_off=%zu w=%d h=%d grid_off=%zu extra=%d", i, c.header_off, c.w, c.h, c.grid_off, c.extra);
    }
    if (candidates.empty()) {
        RCLCPP_WARN(node->get_logger(), "No candidates found automatically. You may try specifying offsets manually or increase search windows.");
        // exit but leave node alive for convenience
        rclcpp::shutdown();
        return 0;
    }

    int pick = 0;
    if (choose_idx >= 0 && choose_idx < (int)candidates.size()) pick = choose_idx;
    // otherwise pick first
    const Candidate chosen = candidates[pick];
    RCLCPP_INFO(node->get_logger(), "Using candidate %d: w=%d h=%d grid_off=%zu", pick, chosen.w, chosen.h, chosen.grid_off);

    // extract raw block
    size_t               w = (size_t)chosen.w;
    size_t               h = (size_t)chosen.h;
    size_t               need = w * h;
    std::vector<uint8_t> raw(need);
    std::copy(data.begin() + chosen.grid_off, data.begin() + chosen.grid_off + need, raw.begin());

    // convert raw bytes to occupancy [-1,0..100]
    std::vector<int8_t> occ(need);
    // heuristic: if bytes in [0..100] -> leave; if 255 present (unknown) map to -1; else map 0..255 -> 0..100
    bool    has255 = false;
    uint8_t maxv = 0;
    for (size_t i = 0; i < need; ++i) {
        if (raw[i] == 255) has255 = true;
        if (raw[i] > maxv) maxv = raw[i];
    }
    if (maxv <= 100) {
        for (size_t i = 0; i < need; ++i) occ[i] = static_cast<int8_t>(raw[i]);  // 0..100
    } else {
        for (size_t i = 0; i < need; ++i) {
            if (raw[i] == 255)
                occ[i] = -1;
            else {
                int v = (int)raw[i];
                int mapped = (int)std::round((double)v * 100.0 / 255.0);
                if (mapped > 100) mapped = 100;
                occ[i] = static_cast<int8_t>(mapped);
            }
        }
    }

    // create OccupancyGrid message
    auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("sancan_map", 10);

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.width = w;
    grid.info.height = h;
    grid.info.resolution = resolution_param;
    grid.info.origin.position.x = 0.0;  // unknown from file; user may edit
    grid.info.origin.position.y = 0.0;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    // data order: OccupancyGrid expects row-major, starting at (0,0) lower-left.
    // Our raw likely stored row-major top-left; we won't flip — user can set invert in RViz if needed.
    grid.data.resize(need);
    for (size_t i = 0; i < need; ++i) grid.data[i] = occ[i];

    // Save PGM and YAML for manual inspect
    try {
        std::vector<uint8_t> pgm_raw(need);
        // make display: 0 free -> 255 white, 100 occ -> 0 black, -1 unknown -> 127 gray
        for (size_t i = 0; i < need; ++i) {
            int8_t  v = occ[i];
            uint8_t out = 127;
            if (v == -1)
                out = 127;
            else {
                int val = (int)v;  // 0..100
                out = static_cast<uint8_t>(std::max(0, std::min(255, 255 - (val * 255 / 100))));
            }
            pgm_raw[i] = out;
        }
        write_pgm("map_candidate.pgm", (int)w, (int)h, pgm_raw);
        write_yaml("map_candidate.yaml", resolution_param, 0.0, 0.0, "map_candidate.pgm");
        RCLCPP_INFO(node->get_logger(), "Written map_candidate.pgm / map_candidate.yaml");
    } catch (...) {
        RCLCPP_WARN(node->get_logger(), "Failed to write PGM/YAML");
    }

    // publish repeatedly so RViz can pick up
    rclcpp::Rate rate(1);
    int          loop = 0;
    while (rclcpp::ok() && loop < 1000) {
        grid.header.stamp = node->now();
        grid.header.frame_id = "map";
        pub->publish(grid);
        rclcpp::spin_some(node);
        rate.sleep();
        ++loop;
    }

    rclcpp::shutdown();
    return 0;
}
