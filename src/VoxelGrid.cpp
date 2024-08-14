#include <open3d/t/geometry/RaycastingScene.h>
#include <spdlog/spdlog.h>

#include "VoxelGrid.h"
#include "Config.h"
#include "VisualizerHelper.h"


inline double sqr_dis(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    return (a(0) - b(0)) * (a(0) - b(0)) + (a(1) - b(1)) * (a(1) - b(1)) + (a(2) - b(2)) * (a(2) - b(2));
}

void VoxelGrid::initialize() {
    /// @brief 初始化体素网格
    // voxel_size = Config["VoxelGrid"]["voxelSize"].as<double>();
    // dilate_size = Config["VoxelGrid"]["dilateSize"].as<size_t>();
    voxel_size = CFG2("VoxelGrid", "voxelSize", double);
    dilate_size = CFG2("VoxelGrid", "dilateSize", size_t);

    bbox = open3d::geometry::AxisAlignedBoundingBox(
        Eigen::Vector3d {
            CFG4("VoxelGrid", "gridBox", "minCorr", 0, double),
            CFG4("VoxelGrid", "gridBox", "minCorr", 1, double),
            CFG4("VoxelGrid", "gridBox", "minCorr", 2, double) },
        Eigen::Vector3d {
            CFG4("VoxelGrid", "gridBox", "maxCorr", 0, double),
            CFG4("VoxelGrid", "gridBox", "maxCorr", 1, double),
            CFG4("VoxelGrid", "gridBox", "maxCorr", 2, double) });
    grid_size = ((bbox.GetMaxBound() - bbox.GetMinBound()) / voxel_size + Eigen::Vector3d::Ones()).cast<int>();
    grid = std::vector<std::vector<std::vector<bool> > >(
        grid_size[0], std::vector<std::vector<bool> >(
            grid_size[1], std::vector<bool>(
                grid_size[2], false
            )
        )
    );
    o3d_grid.voxel_size_ = voxel_size;
    o3d_grid.origin_ = bbox.GetMinBound() - Eigen::Vector3d::Ones() * voxel_size * 0.5;
}

void VoxelGrid::occupy_by_mesh(std::shared_ptr<const open3d::geometry::TriangleMesh> mesh, std::shared_ptr<const open3d::geometry::TriangleMesh> mesh_f, Eigen::Vector3d lidar_pos) {
    /// @brief 利用模型占据体素网格
    spdlog::info("Occupying voxel grid by mesh");

    open3d::t::geometry::RaycastingScene scene, scene_r;
    scene.AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_f));
    scene_r.AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*mesh));
    // double thres = Config["VoxelGrid"]["meshExpand"].as<double>();
    double thres = CFG2("VoxelGrid", "meshExpand", double);

    Eigen::MatrixXf P(grid_size[0] * grid_size[1] * grid_size[2], 3);   // 查询点集
    Eigen::MatrixXf Q(grid_size[0] * grid_size[1] * grid_size[2], 6);   // 查询光线
    for (int i = 0; i < grid_size[0]; ++i)
    for (int j = 0; j < grid_size[1]; ++j)
    for (int k = 0; k < grid_size[2]; ++k) {
        P.row(i * grid_size[1] * grid_size[2] + j * grid_size[2] + k)
            = get_grid_center({i, j, k}).cast<float>();
        Q.row(i * grid_size[1] * grid_size[2] + j * grid_size[2] + k)(Eigen::seq(0, 2)) = lidar_pos.cast<float>();
        Q.row(i * grid_size[1] * grid_size[2] + j * grid_size[2] + k)(Eigen::seq(3, 5)) = (get_grid_center({i, j, k}) - lidar_pos).cast<float>();
    }

    open3d::core::Tensor P_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(P);
    open3d::core::Tensor Q_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(Q);

    // 计算每个点到 mesh 的距离, 低于阈值的点被认为是在 mesh 内部而被占据
    open3d::core::Tensor rslt = scene.ComputeSignedDistance(P_tensor);
    for (int i = 0; i < grid_size[0]; ++i)
    for (int j = 0; j < grid_size[1]; ++j)
    for (int k = 0; k < grid_size[2]; ++k) {
        if (rslt[i * grid_size[1] * grid_size[2] + j * grid_size[2] + k].Item<float>() < thres) {
            occupy(Eigen::Vector3i{i, j, k});
        }
    }

    // 计算每条由 雷达位置 到 网格中心 的线段是否与 mesh 相交, 相交则认为该网格被占据
    std::unordered_map<std::string, open3d::core::Tensor> rslt_r = scene_r.CastRays(Q_tensor);
    open3d::core::Tensor rslt_hit = rslt_r["t_hit"];
    for (int i = 0; i < grid_size[0]; ++i)
    for (int j = 0; j < grid_size[1]; ++j)
    for (int k = 0; k < grid_size[2]; ++k) {
        // spdlog::info("({}, {}, {}): {}", i, j, k, rslt_hit[i * grid_size[1] * grid_size[2] + j * grid_size[2] + k].Item<float>());
        // if (rslt_hit[i * grid_size[1] * grid_size[2] + j * grid_size[2] + k].Item<float>()
        //     < (get_grid_center({i, j, k}) - lidar_pos).norm()) {
        if (rslt_hit[i * grid_size[1] * grid_size[2] + j * grid_size[2] + k].Item<float>() < 1) {
            // grid[i][j][k] = true;
            dilate_occupy(Eigen::Vector3i{i, j, k}, {1, 0, 0});
        }
    }

    double z_max = bbox.max_bound_(2);
    z_map = Eigen::MatrixXf::Zero(grid_size[0], grid_size[1]);
    Eigen::MatrixXf R(grid_size[0] * grid_size[1], 6);   // 查询光线
    for (int i = 0; i < grid_size[0]; ++i)
    for (int j = 0; j < grid_size[1]; ++j) {
        R.row(i * grid_size[1] + j)(Eigen::seq(0, 1)) = get_grid_center({i, j, 0})(Eigen::seq(0, 1)).cast<float>();
        R.row(i * grid_size[1] + j)(2) = z_max;
        R.row(i * grid_size[1] + j)(Eigen::seq(3, 5)) = Eigen::Vector3f {0, 0, -1};
    }
    open3d::core::Tensor R_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(R);
    std::unordered_map<std::string, open3d::core::Tensor> rslt_r2 = scene_r.CastRays(R_tensor);
    open3d::core::Tensor rslt_hit2 = rslt_r2["t_hit"];
    for (int i = 0; i < grid_size[0]; ++i)
    for (int j = 0; j < grid_size[1]; ++j) {
        z_map(i, j) = z_max - rslt_hit2[i * grid_size[1] + j].Item<float>();
    }

    spdlog::info("Occupying voxel grid by mesh done");
}

void VoxelGrid::occupy_by_pc(std::shared_ptr<const open3d::geometry::PointCloud> pc_aligned,
                             std::shared_ptr<const open3d::geometry::TriangleMesh> mesh) {
    /// @brief 利用点云占据体素网格
    spdlog::info("Occupying voxel grid by point cloud");
    open3d::geometry::PointCloud pc_filtered, pc_rest;
    for (const auto &pt : pc_aligned->points_) {
        if (is_occupied(pt)) {
            pc_rest.points_.push_back(pt);
        } else {
            pc_filtered.points_.push_back(pt);
        }
    }
    pc_rest.PaintUniformColor({1, 0, 0});
    auto new_pc = vis::crop_point_cloud(
        std::make_shared<open3d::geometry::PointCloud>(pc_filtered),
        std::make_shared<open3d::geometry::PointCloud>(pc_rest),
        mesh);
    for (const auto &pt : new_pc.points_)
        dilate_occupy(pt, {0, 1, 0});
    spdlog::info("Occupying voxel grid by point cloud done");
}

void VoxelGrid::calculate_zmap(std::shared_ptr<const open3d::geometry::TriangleMesh> mesh) {
    // @brief 计算高度图
    spdlog::info("Calculating zmap");
    double z_max = bbox.max_bound_(2);
    open3d::t::geometry::RaycastingScene scene_r;
    scene_r.AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*mesh));
    z_map = Eigen::MatrixXf::Zero(grid_size[0], grid_size[1]);
    Eigen::MatrixXf Q(grid_size[0] * grid_size[1], 6);   // 查询光线
    for (int i = 0; i < grid_size[0]; ++i)
    for (int j = 0; j < grid_size[1]; ++j) {
        Q.row(i * grid_size[1] + j)(Eigen::seq(0, 1)) = get_grid_center({i, j, 0})(Eigen::seq(0, 1)).cast<float>();
        Q.row(i * grid_size[1] + j)(2) = z_max;
        Q.row(i * grid_size[1] + j)(Eigen::seq(3, 5)) = Eigen::Vector3f {0, 0, -1};
    }
    open3d::core::Tensor Q_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(Q);
    std::unordered_map<std::string, open3d::core::Tensor> rslt_r = scene_r.CastRays(Q_tensor);
    open3d::core::Tensor rslt_hit = rslt_r["t_hit"];
    for (int i = 0; i < grid_size[0]; ++i)
    for (int j = 0; j < grid_size[1]; ++j) {
        z_map(i, j) = z_max - rslt_hit[i * grid_size[1] + j].Item<float>();
    }
    spdlog::info("Calculating zmap done");
}

void VoxelFilter::initialize()
{
    /// @brief 初始化体素网格
    // voxel_size = Config["PointCloud"]["voxelFilter"]["voxelSize"].as<double>();
    voxel_size = CFG3("PointCloud", "voxelFilter", "voxelSize", double);
    bbox = open3d::geometry::AxisAlignedBoundingBox(
        Eigen::Vector3d {
            CFG4("VoxelGrid", "gridBox", "minCorr", 0, double),
            CFG4("VoxelGrid", "gridBox", "minCorr", 1, double),
            CFG4("VoxelGrid", "gridBox", "minCorr", 2, double) },
        Eigen::Vector3d {
            CFG4("VoxelGrid", "gridBox", "maxCorr", 0, double),
            CFG4("VoxelGrid", "gridBox", "maxCorr", 1, double),
            CFG4("VoxelGrid", "gridBox", "maxCorr", 2, double) });
    grid_size = ((bbox.GetMaxBound() - bbox.GetMinBound()) / voxel_size + Eigen::Vector3d::Ones()).cast<int>();
    grid = std::vector<std::vector<std::vector<std::pair<size_t, size_t>>>>(grid_size(0) + 1,
            std::vector<std::vector<std::pair<size_t, size_t>>>(grid_size(1) + 1,
            std::vector<std::pair<size_t, size_t>>(grid_size(2) + 1, { 0, 0 })));
}

std::vector<Eigen::Vector3d> VoxelFilter::filter(const std::vector<Eigen::Vector3d>& pcd, double rate)
{
    /// @brief 体素滤波
    if (pcd.empty())
        return {};
    // clean filter_voxel
    for (auto &i: grid) {
        for (auto &j: i) {
            j.assign(j.size(), std::make_pair((size_t) 0, (size_t) 0));
        }
    }

    std::vector<Eigen::Vector3d> filtered;

    for (auto& point : pcd)
    {
        if ((point.array() >= bbox.max_bound_.array()).any() || (point.array() < bbox.min_bound_.array()).any())
            continue; // 超出范围的点默认为占据以过滤
        Eigen::Vector3d pt_grid = (point - bbox.min_bound_) / voxel_size;
        Eigen::Vector3i idx = pt_grid.cast<int>();
        grid[idx(0)][idx(1)][idx(2)].second++;
        if (grid[idx(0)][idx(1)][idx(2)].first / double(grid[idx(0)][idx(1)][idx(2)].second) <= rate) {
            filtered.push_back(point);
            grid[idx(0)][idx(1)][idx(2)].first++;
        }
    }
    return filtered;
}
