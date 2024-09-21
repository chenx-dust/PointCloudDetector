#pragma once

#include <open3d/Open3D.h>

#include "PcContext.h"

class PcVisualizer {
    std::shared_ptr<PcContext> ctx;

    open3d::visualization::VisualizerWithKeyCallback vis;

    std::shared_ptr<open3d::geometry::TriangleMesh> lidar_mesh;
    std::shared_ptr<open3d::geometry::TriangleMesh> camera_mesh;
    std::shared_ptr<open3d::geometry::TriangleMesh> axis_mesh;

    std::shared_ptr<open3d::geometry::PointCloud> pc_vis;
    std::vector<std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox>> aabbs_vis;
    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> spheres_vis;

    size_t tm_size;
    size_t vis_update_int, speed_int;

    void mesh_create();

public:
    void Initialize(std::shared_ptr<PcContext> ctx);
    void Loop();
};