
#pragma once

/// VisualizerHelper
/// 可视化相关的工具

#include <open3d/Open3D.h>
#include <open3d/visualization/visualizer/RenderOptionWithEditing.h>

/// 这是一个继承自VisualizerWithEditing的类，用于实现点云的选择功能
namespace vis {

using open3d::visualization::SelectionPolygon;
using open3d::visualization::ViewControl;
using open3d::visualization::VisualizerWithEditing;

class VisualizerSelect : public VisualizerWithEditing {
    open3d::geometry::PointCloud pcd2rtn;
    std::shared_ptr<const open3d::geometry::PointCloud> pcd_rest;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_rest_vis;
    bool rest_vis = false;

    std::vector<size_t> pCropInRectangle(const SelectionPolygon& polygon,
        const std::vector<Eigen::Vector3d>& input, const ViewControl& view);

    std::vector<size_t> pCropInPolygon(const SelectionPolygon& polygon,
        const std::vector<Eigen::Vector3d>& input, const ViewControl& view);

public:
    void KeyPressCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    void AddRestPc(std::shared_ptr<const open3d::geometry::PointCloud> geometry_ptr);

    bool AddGeometryOnlyView(std::shared_ptr<const open3d::geometry::Geometry> geometry_ptr);

    open3d::geometry::PointCloud&& GetEditedPointCloud();
};

/// 从点云中选择点
std::vector<size_t> select_points(std::shared_ptr<const open3d::geometry::PointCloud> pcd);

/// 可视化地裁切点云
open3d::geometry::PointCloud crop_point_cloud(std::shared_ptr<const open3d::geometry::PointCloud> pcd, std::shared_ptr<const open3d::geometry::PointCloud> pc_rest, std::shared_ptr<const open3d::geometry::TriangleMesh> mesh);
}