#pragma once

#include <atomic>
#include <deque>
#include <utility>
#include <memory>

#include <Eigen/Eigen>
#include <open3d/Open3D.h>

#include "PcContext.h"
#include "VoxelGrid.h"

/// @struct 点云主循环类
class PcDetector {
    std::shared_ptr<PcContext> ctx;

    VoxelGrid voxel_grid;
    VoxelFilter voxel_filter;

    std::deque<Eigen::Vector3d> process_queue;
    std::deque<size_t> size_queue;

    size_t buffer_size;
    size_t sample_size;
    // 过滤
    size_t expected_max_points;
    size_t max_density;
    double eps;
    double min_points_k;
    size_t min_points;
    bool use_diff;
    Eigen::Vector3d lidar_pos;

    void pub_targets();
    void pub_trans();
    std::shared_ptr<open3d::geometry::PointCloud> sample_pc();

public:
    void Initialize(std::shared_ptr<PcContext> ctx);
    void Loop();
};