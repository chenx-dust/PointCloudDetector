#include "PcVisualizer.h"

#include <fstream>
#include <nlohmann/json.hpp>

using open3d::geometry::PointCloud;
using open3d::geometry::TriangleMesh;

void PcVisualizer::Initialize(std::shared_ptr<PcContext> ctx_)
{
    vis_update_int = CFG2("Visualizer", "updateInterval", size_t);
    speed_int = CFG2("Visualizer", "speedInterval", size_t);

    ctx = ctx_;
    vis.RegisterKeyCallback(GLFW_KEY_D, [&](open3d::visualization::Visualizer*) {
        spdlog::info("Debug output");
        nlohmann::json data;
        for (const auto& target : ctx->targetMap.target_map) {
            if (target.discarded)
                continue;
            nlohmann::json target_json;
            target_json["id"] = target.id;
            target_json["X"] = target.kf.X;
            target_json["P"] = target.kf.P.rowwise();
            data["targets"].push_back(target_json);
        }
        spdlog::info("{}", data.dump());
        std::ofstream fout("debug.json");
        fout << data.dump();
        fout.close();
        return true;
    });
    vis.RegisterKeyCallback(GLFW_KEY_S, [&](open3d::visualization::Visualizer*) {
        spdlog::info("Saving points...");
        std::ofstream fout("points.csv");
        for (auto& pt : ctx->pc->points_)
            fout << pt(0) << "," << pt(1) << "," << pt(2) << std::endl;
        fout.close();
        return true;
    });
    vis.RegisterKeyCallback(GLFW_KEY_P, [&](open3d::visualization::Visualizer*) {
        ctx->is_paused = !ctx->is_paused;
        ctx->is_step = false;
        spdlog::info("Toogle Pause: {}", ctx->is_paused);
        return true;
    });
    vis.RegisterKeyCallback(GLFW_KEY_T, [&](open3d::visualization::Visualizer*) {
        ctx->is_step = false;
        spdlog::info("Single Step");
        return true;
    });
    vis.CreateVisualizerWindow("Point Cloud Prediction", 1920, 1080);
    vis.BuildUtilities();
    vis.UpdateWindowTitle();

    mesh_create();
    vis.AddGeometry(ctx->mesh);
    vis.AddGeometry(lidar_mesh);
    vis.AddGeometry(camera_mesh);
    vis.AddGeometry(axis_mesh);

    pc_vis = std::make_shared<PointCloud>(*ctx->pc);
    vis.AddGeometry(pc_vis);
    for (const auto& target : ctx->targetMap.target_map) {
        aabbs_vis.push_back(std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(*target.aabb));
        spheres_vis.push_back(std::make_shared<TriangleMesh>(*target.mesh));
        vis.AddGeometry(aabbs_vis.back());
        vis.AddGeometry(spheres_vis.back());
    }
    tm_size = ctx->targetMap.target_map.size();
}

void PcVisualizer::mesh_create()
{
    lidar_mesh = TriangleMesh::CreateCylinder(100, 500);
    camera_mesh = TriangleMesh::CreateCylinder(100, 500);
    Eigen::Matrix3d xz = Eigen::Matrix3d { { 0, 0, 1 }, { 0, 1, 0 }, { -1, 0, 0 } };
    lidar_mesh->Rotate(xz, Eigen::Vector3d::Zero());
    lidar_mesh->Transform(ctx->trans.T);
    camera_mesh->Transform(ctx->trans.T * ctx->trans.E_0.inverse());
    lidar_mesh->PaintUniformColor({ 0, 0, 1 });
    camera_mesh->PaintUniformColor({ 1, 0, 0 });
    axis_mesh = TriangleMesh::CreateCoordinateFrame(1000);
}

void PcVisualizer::Loop()
{
    while (!ctx->stop_signal) {
        static size_t report_cnt = 0;
        {
            std::lock_guard<std::mutex> lock_(ctx->vis_mutex);
            *pc_vis = *ctx->pc;
            for (size_t i = 0; i < tm_size; ++i) {
                *aabbs_vis[i] = *ctx->targetMap.target_map[i].aabb;
                *spheres_vis[i] = *ctx->targetMap.target_map[i].mesh;
            }
            if (report_cnt >= speed_int) {
                spdlog::info("process speed: {} pt/s, rate: {}",
                    ctx->pt_cnt * 1000 / (vis_update_int * speed_int),
                    ctx->pt_cnt * 1000 / (vis_update_int * speed_int * 452000.0));
                report_cnt = 0;
                ctx->pt_cnt = 0;
            }
        }
        report_cnt++;
        vis.PollEvents();
        vis.UpdateGeometry();
        vis.UpdateRender();
        std::this_thread::sleep_for(std::chrono::milliseconds(vis_update_int));
    }
    vis.DestroyVisualizerWindow();
}
