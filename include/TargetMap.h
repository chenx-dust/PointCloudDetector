
#pragma once

/// TargetMap
/// 这是用于储存更新定位目标的类，包括目标的卡尔曼滤波、位置，以及用于可视化的一些东西

#include <cstddef>
#include <deque>
#include <memory>
#include <open3d/Open3D.h>
#include <queue>
#include <spdlog/spdlog.h>
#include <vector>

#include "Config.h"
#include "KalmanFilter.h"
#include "VoxelGrid.h"

#define TM_FAILED_INSERT -1
#define TM_NEED_SEPERATE -2

struct Target {
    size_t id; // 目标 id
    size_t lost_time; // 跟丢计数
    bool discarded; // 是否被丢弃 (被丢弃的目标会用作下一次插入)
    KalmanFilter kf; // 卡尔曼滤波器
    size_t pt_num; // 单次更新中的点数量 (为 0 意味着目标没发生更新)
    Eigen::Vector3d grav; // 单次更新中的重心
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> aabb;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
};

class TargetMap {
private:
    size_t inc_id = 0; // 自增 id
    std::function<Eigen::Vector3d(const Eigen::Vector3d&)> project_func;
    void element_update(size_t key, const open3d::geometry::AxisAlignedBoundingBox& aabb, size_t pt_num, Eigen::Vector3d grav);
    void reset(size_t key, bool is_init = false);
    void pre_update();
    void post_update();
    size_t new_target(const open3d::geometry::AxisAlignedBoundingBox& aabb, size_t pt_num, Eigen::Vector3d grav);
    void combine(size_t new_key, std::vector<size_t> old_keys);
    void combine_force(size_t new_key, std::vector<size_t> old_keys);
    void seperate(const open3d::geometry::AxisAlignedBoundingBox& aabb, const open3d::geometry::PointCloud& pc);

public:
    void initialize(const VoxelGrid& vg);

    static void cluster(const open3d::geometry::PointCloud& pc, double eps_, size_t min_points_, std::vector<int>& labels, std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& pcs,
        open3d::geometry::PointCloud& pc_noise, std::vector<open3d::geometry::AxisAlignedBoundingBox>& aabbs, std::vector<size_t>& pt_num, std::vector<Eigen::Vector3d>& grav);

    static void cluster_d(const open3d::geometry::PointCloud& pc, const Eigen::Vector3d& zero_pos, double eps_, size_t min_points_k,
        std::vector<int>& labels, std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& pcs, open3d::geometry::PointCloud& pc_noise,
        std::vector<open3d::geometry::AxisAlignedBoundingBox>& aabbs, std::vector<size_t>& pt_num, std::vector<Eigen::Vector3d>& grav);

    std::vector<Target> target_map;
    std::queue<size_t> discarded_queue;

    size_t push(const open3d::geometry::AxisAlignedBoundingBox& aabb, size_t pt_num, Eigen::Vector3d grav, const open3d::geometry::PointCloud& pc, bool no_strict = false);
    std::vector<int> update(const std::vector<open3d::geometry::AxisAlignedBoundingBox>& aabbs,
        const std::vector<size_t>& pt_nums,
        const std::vector<Eigen::Vector3d>& grav,
        const open3d::geometry::PointCloud& pc);
    void loose_query(const open3d::geometry::PointCloud& pc);
};
