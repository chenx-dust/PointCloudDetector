#include "VisualizerHelper.h"
#include <spdlog/spdlog.h>
#include <fstream>

namespace vis {
std::vector<size_t> select_points(std::shared_ptr<const open3d::geometry::PointCloud> pcd)
{
    spdlog::info("Select points...");
    spdlog::info("  Use [shift + left click] to pick points.");
    spdlog::info("  Use [shift + right click] to undo point picking.");
    spdlog::info("  After picking points, press 'Q' to close the window.");

    open3d::visualization::VisualizerWithEditing vis_;
    vis_.CreateVisualizerWindow("Select Points", 1920, 1080);
    vis_.AddGeometry(pcd);
    vis_.Run();
    vis_.DestroyVisualizerWindow();
    return vis_.GetPickedPoints();
}

open3d::geometry::PointCloud crop_point_cloud(std::shared_ptr<const open3d::geometry::PointCloud> pcd, std::shared_ptr<const open3d::geometry::PointCloud> pc_rest, std::shared_ptr<const open3d::geometry::TriangleMesh> mesh)
{
    spdlog::info("Cropping point cloud...");
    spdlog::info("[[UI Instruction]]");
    spdlog::info("=======Basic Usage=======");
    spdlog::info("  Press 'K' to lock view and to select.");
    spdlog::info("  Use [left click + drag] to select rectangle.");
    spdlog::info("  Use [ctrl + left click + drag] to select polygon.");
    spdlog::info("  Press 'C' to crop the point cloud.");
    spdlog::info("  Press 'Q' to close the window.");
    spdlog::info("=====Advanced Usage=====");
    spdlog::info("  Press 'G' to toggle showing original points.");
    spdlog::info("  Press 'Z' to enter orthogonal view along Z axis, press again to flip.");
    spdlog::info("  Press 'F' to return free view.");
    VisualizerSelect vis_;
    vis_.CreateVisualizerWindow("Crop Point Cloud", 1920, 1080);
    vis_.AddGeometry(pcd);
    vis_.AddRestPc(pc_rest);
    vis_.AddGeometryOnlyView(mesh);
    vis_.Run();
    vis_.DestroyVisualizerWindow();
    open3d::geometry::PointCloud rtn = vis_.GetEditedPointCloud();
    //    open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(rtn)});
    return rtn;
}

}