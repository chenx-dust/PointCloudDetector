/// 这是一个继承自VisualizerWithEditing的类，用于实现点云的选择功能

#include "VisualizerHelper.h"
#include <spdlog/spdlog.h>

namespace vis {
std::vector<size_t> VisualizerSelect::pCropInRectangle(const SelectionPolygon& polygon,
    const std::vector<Eigen::Vector3d>& input, const ViewControl& view)
{
    std::vector<size_t> output_index;
    Eigen::Matrix4d mvp_matrix = view.GetMVPMatrix().cast<double>();
    double half_width = (double)view.GetWindowWidth() * 0.5;
    double half_height = (double)view.GetWindowHeight() * 0.5;
    auto min_bound = polygon.GetMinBound();
    auto max_bound = polygon.GetMaxBound();
    //        utility::ProgressBar progress_bar((int64_t)input.size(),
    //                                          "Cropping geometry: ");
    for (size_t i = 0; i < input.size(); i++) {
        //            ++progress_bar;
        const auto& point = input[i];
        Eigen::Vector4d pos = mvp_matrix * Eigen::Vector4d(point(0), point(1), point(2), 1.0);
        if (pos(3) == 0.0)
            break;
        pos /= pos(3);
        double x = (pos(0) + 1.0) * half_width;
        double y = (pos(1) + 1.0) * half_height;
        if (x >= min_bound(0) && x <= max_bound(0) && y >= min_bound(1) && y <= max_bound(1)) {
            output_index.push_back(i);
        }
    }
    return output_index;
}

std::vector<size_t> VisualizerSelect::pCropInPolygon(const SelectionPolygon& polygon,
    const std::vector<Eigen::Vector3d>& input, const ViewControl& view)
{
    std::vector<size_t> output_index;
    Eigen::Matrix4d mvp_matrix = view.GetMVPMatrix().cast<double>();
    double half_width = (double)view.GetWindowWidth() * 0.5;
    double half_height = (double)view.GetWindowHeight() * 0.5;
    std::vector<double> nodes;
    //        utility::ProgressBar progress_bar((int64_t)input.size(),
    //                                          "Cropping geometry: ");
    for (size_t k = 0; k < input.size(); k++) {
        //            ++progress_bar;
        const auto& point = input[k];
        Eigen::Vector4d pos = mvp_matrix * Eigen::Vector4d(point(0), point(1), point(2), 1.0);
        if (pos(3) == 0.0)
            break;
        pos /= pos(3);
        double x = (pos(0) + 1.0) * half_width;
        double y = (pos(1) + 1.0) * half_height;
        nodes.clear();
        for (size_t i = 0; i < polygon.polygon_.size(); i++) {
            size_t j = (i + 1) % polygon.polygon_.size();
            if ((polygon.polygon_[i](1) < y && polygon.polygon_[j](1) >= y) || (polygon.polygon_[j](1) < y && polygon.polygon_[i](1) >= y)) {
                nodes.push_back(polygon.polygon_[i](0) + (y - polygon.polygon_[i](1)) / (polygon.polygon_[j](1) - polygon.polygon_[i](1)) * (polygon.polygon_[j](0) - polygon.polygon_[i](0)));
            }
        }
        std::sort(nodes.begin(), nodes.end());
        auto loc = std::lower_bound(nodes.begin(), nodes.end(), x);
        if (std::distance(nodes.begin(), loc) % 2 == 1) {
            output_index.push_back(k);
        }
    }
    return output_index;
}
void VisualizerSelect::KeyPressCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_RELEASE || (key != GLFW_KEY_C && key != GLFW_KEY_G)) {
        VisualizerWithEditing::KeyPressCallback(window, key, scancode, action, mods);
        return;
    }
    if (key == GLFW_KEY_G) {
        if (rest_vis) {
            pcd_rest_vis->points_.clear();
            pcd_rest_vis->colors_.clear();
        }
        else {
            pcd_rest_vis->points_ = pcd_rest->points_;
            pcd_rest_vis->colors_ = pcd_rest->colors_;
        }
        UpdateGeometry(pcd_rest_vis);
        rest_vis = !rest_vis;
        // UpdateRender();
        return;
    }
    auto& view_control = (open3d::visualization::ViewControlWithEditing&)(*view_control_ptr_);
    if (view_control.IsLocked() && selection_polygon_ptr_) {
        if (editing_geometry_ptr_ && editing_geometry_ptr_->GetGeometryType() == open3d::geometry::Geometry::GeometryType::PointCloud) {
            // glfwMakeContextCurrent(window_);
            open3d::geometry::PointCloud& pcd = (open3d::geometry::PointCloud&)*editing_geometry_ptr_;
            //                if (std::shared_ptr<open3d::geometry::PointCloud> pcd_ptr =
            //                        selection_polygon_ptr_->CropPointCloud(
            //                                pcd, view_control)) {
            //                    pcd = *pcd_ptr;
            //                } else {
            //                    open3d::utility::LogError(
            //                            "Internal error: CropPointCloud returned "
            //                            "nullptr.");
            //                }
            // 以下为增加代码
            switch (selection_polygon_ptr_->polygon_type_) {
            case open3d::visualization::SelectionPolygon::SectionPolygonType::Rectangle:
                //                        return CropPointCloudInRectangle(input, view);
                pcd = *pcd.SelectByIndex(
                    pCropInRectangle(*selection_polygon_ptr_, pcd.points_, *view_control_ptr_), true);
                break;
            case open3d::visualization::SelectionPolygon::SectionPolygonType::Polygon:
                //                        return CropPointCloudInPolygon(input, view);
                pcd = *pcd.SelectByIndex(
                    pCropInPolygon(*selection_polygon_ptr_, pcd.points_, *view_control_ptr_), true);
                break;
            case open3d::visualization::SelectionPolygon::SectionPolygonType::Unfilled:
            default:
                break;
            }
            pcd2rtn = pcd;
            // 以上为增加代码
            editing_geometry_renderer_ptr_->UpdateGeometry();
            view_control.ToggleLocking();
            InvalidateSelectionPolygon();
            InvalidatePicking();

            // 模拟按下 K 键，以便在编辑后立即进入编辑模式
            VisualizerWithEditing::KeyPressCallback(window, GLFW_KEY_K, scancode, action, mods);
        }
    } else {
        Visualizer::KeyPressCallback(window, key, scancode, action,
            mods);
    }
}
bool VisualizerSelect::AddGeometryOnlyView(std::shared_ptr<const open3d::geometry::Geometry> geometry_ptr)
{
    return Visualizer::AddGeometry(geometry_ptr);
}
void VisualizerSelect::AddRestPc(std::shared_ptr<const open3d::geometry::PointCloud> geometry_ptr)
{
    pcd_rest = geometry_ptr;
    pcd_rest_vis = std::make_shared<open3d::geometry::PointCloud>();
    Visualizer::AddGeometry(pcd_rest_vis);
}
open3d::geometry::PointCloud&& VisualizerSelect::GetEditedPointCloud()
{
    return std::move(pcd2rtn);
}
}
