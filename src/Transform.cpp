#include "Transform.h"
#include "VisualizerHelper.h"
#include "Config.h"
#include "Clustering.h"
#include <nlohmann/json.hpp>

using open3d::geometry::TriangleMesh;
using open3d::geometry::PointCloud;
using open3d::geometry::AxisAlignedBoundingBox;
typedef std::shared_ptr<TriangleMesh> TriangleMeshPtr;
typedef std::shared_ptr<PointCloud> PointCloudPtr;


Eigen::Matrix4d manual_trans(PointCloudPtr pc2align, PointCloudPtr mesh_pc) {
    /// @brief 手动配准获取初始变换矩阵
    auto picked_pc = vis::select_points(pc2align);
    auto picked_mesh = vis::select_points(mesh_pc);
    assert(picked_pc.size() >= 3 && picked_mesh.size() >= 3);
    assert(picked_pc.size() == picked_mesh.size());
    std::vector<Eigen::Vector2i> correspondences;
    for (size_t i = 0; i < picked_pc.size(); ++i)
        correspondences.emplace_back(picked_pc[i], picked_mesh[i]);
    open3d::pipelines::registration::TransformationEstimationPointToPoint pointToPoint;
    return pointToPoint.ComputeTransformation(*pc2align, *mesh_pc, correspondences);
}

Eigen::Matrix4d read_from_config() {
    /// @brief 读取参数文件获取初始变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
        T(i, j) = CFG5("TransForm", "localTrans", "rotMat", i, j, double);
    for (int i = 0; i < 3; ++i)
        T(i, 3) = CFG4("TransForm", "localTrans", "transVec", i, double);
    return T;
}

Eigen::Matrix4d read_e0() {
    /// @brief 读取参数文件获取相机到雷达的变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
        T(i, j) = CFG4("TransForm", "cam2LadarTrans", i, j, double);
    return T;
}

AxisAlignedBoundingBox get_crop_box() {
    return AxisAlignedBoundingBox({
            CFG5("TransForm", "autoAlign", "cropBox", "minCorr", 0, double),
            CFG5("TransForm", "autoAlign", "cropBox", "minCorr", 1, double),
            CFG5("TransForm", "autoAlign", "cropBox", "minCorr", 2, double)
        },{
            CFG5("TransForm", "autoAlign", "cropBox", "maxCorr", 0, double),
            CFG5("TransForm", "autoAlign", "cropBox", "maxCorr", 1, double),
            CFG5("TransForm", "autoAlign", "cropBox", "maxCorr", 2, double)
        });
}

Eigen::Matrix4d read_from_mqtt(mqtt::async_client& client)
{
    Eigen::Matrix4d rtn = Eigen::Matrix4d::Identity();
    client.start_consuming();
    client.subscribe("pnp_trans", 1);
    nlohmann::json msg = nlohmann::json::parse(client.consume_message()->get_payload_str());
    for (size_t i = 0; i < 3; ++i)
    for (size_t j = 0; j < 3; ++j)
        rtn(i, j) = msg["rmat"][i][j].get<double>();
    for (size_t i = 0; i < 3; ++i)
        rtn(i, 3) = msg["tvec_mm"][i].get<double>();
    client.stop_consuming();
    return rtn;
}

void remove_outlier(PointCloudPtr pc2align, double radius)
{
    for (auto& pt : pc2align->points_) {
        if (pt.squaredNorm() > radius * radius) {
            pt = Eigen::Vector3d::Zero();
        }
    }
}

void Transform::initialize(PointCloudPtr pc2align, TriangleMeshPtr mesh, mqtt::async_client& client)
{
    /// @brief 初始化变换矩阵
    /// @note 注意 pc2align 会被变换

    /* 当前有以下几种获取变换的方式 */
    // 1. 如果 manualMode 为 true, 则初始变换由手动标定给出
    // 2. 如果 localTrans.enable 为 true, 则初始变换由参数文件预设给出
    // 3. 如果 localTrans.enable 为 false, 则初始变换从服务器接受

    // 移除离群点, 以防止手动配准困难
    remove_outlier(pc2align, CFG2("TransForm", "outlierRadius", double));

    E_0 = read_e0();
    Transform::print_transform(E_0);
    // 获取初始变换
    if (CFG3("TransForm", "manualAlign", "enable", bool)) {
        spdlog::info("Trans init mode: manual");
        PointCloudPtr mesh_pc = mesh->SamplePointsUniformly(
            CFG3("TransForm", "manualAlign", "meshSamplePoints", size_t));
        // 划分颜色便于观察
        mesh_pc->PaintUniformColor({ 1, 1, 1 });
        for (size_t i = 0; i < mesh_pc->points_.size(); ++i) {
            double z_clr = int(mesh_pc->points_[i](2)) % 1000 / 1000.0;
            if (mesh_pc->points_[i](0) < 14000)
                mesh_pc->colors_[i] = { 1, z_clr, z_clr };
            else
                mesh_pc->colors_[i] = { z_clr, z_clr, 1 };
        }
        T = manual_trans(pc2align, mesh_pc);

    } else {
        if (CFG3("TransForm", "localTrans", "enable", bool)) {
            spdlog::info("Trans init mode: local");
            spdlog::warn("PLEASE MAKE SURE YOU DONT NEED EPNP RESULT!");
            T = read_from_config();
        } else {
            spdlog::info("Trans init mode: remote");
            T = read_from_mqtt(client);
        }
        spdlog::info("Original Trans:");
        Transform::print_transform(T);
        T = T.inverse() * E_0;
    }
    pc2align->Transform(T);

    // 自动配准
    if (CFG3("TransForm", "autoAlign", "enable", bool)) {
        spdlog::info("Auto aligning...");
        // 粗略变换
        // 裁剪点云
        AxisAlignedBoundingBox crop_box = get_crop_box();
        PointCloudPtr pc2align_crop = pc2align->Crop(crop_box);
        // open3d::visualization::DrawGeometries({pc2align_crop, mesh});
        PointCloudPtr mesh_pc = mesh->SamplePointsUniformly(
            CFG3("TransForm", "autoAlign", "meshSamplePoints", size_t));
        
        AxisAlignedBoundingBox mesh_box = mesh->GetAxisAlignedBoundingBox();
        mesh_box.min_bound_(2) = -1;    // 保证模型切掉下表面, 留下地面
        mesh_pc = mesh_pc->Crop(mesh_box);
        // open3d::visualization::DrawGeometries({mesh_pc});
        // 迭代
        auto reg_p2l = open3d::pipelines::registration::RegistrationICP(
            *pc2align_crop, *mesh_pc,
            CFG3("TransForm", "autoAlign", "maxCorrDist", double),
            Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationPointToPlane());
        pc2align->Transform(reg_p2l.transformation_);
        T = reg_p2l.transformation_ * T;
        // spdlog::info("Auto Aligned Trans:");
        // Transform::print_transform(T);
    } else {
        spdlog::warn("AUTO ALIGN DISABLED!");
    }

    spdlog::info("Camera Trans:");
    Transform::print_transform((T * E_0.inverse()).inverse());

    spdlog::info("Showing aligned point cloud...");
    open3d::visualization::DrawGeometries({pc2align, mesh}, "Aligned Point Cloud", 1920, 1080);
}