#include "PcContext.h"

using open3d::geometry::TriangleMesh;

void ReconnectCallback::connection_lost(const std::string& cause)
{
    spdlog::warn("Communication: Connection lost because {}", cause);
    try {
        ctx->commu_client->reconnect()->wait();
        spdlog::info("Communication: Reconnected");
    } catch (const mqtt::exception& exc) {
        spdlog::error("Communication: Reconnect failed: {}", exc.what());
        ctx->commu_enable = false;
    }
}

std::unique_lock<std::mutex> PcContext::ReadBuffer::wait_and_lock(std::atomic_bool& stop_signal)
{
    std::unique_lock<std::mutex> lock(mutex);
    while (!flag && !stop_signal)
        cond_var.wait(lock);
    return lock;
}

void PcContext::Initialize()
{
    // 通信初始化
    commu_enable = CFG2("Communication", "enable", bool);
    reconnect_callback = ReconnectCallback(this);
    if (commu_enable) {
        try {
            commu_client = std::make_shared<mqtt::async_client>(CFG2("Communication", "address", std::string), "pointcloud");
            commu_client->set_callback(reconnect_callback);
            commu_client->connect()->wait();
            spdlog::info("Communication: Connected to {}", CFG2("Communication", "address", std::string));
            commu_topic_detection = mqtt::topic::create(*commu_client, "pc_detected", 0, false);
            // commu_topic_trans = mqtt::topic::create(*commu_client, "aligned_trans", 1, true);
        } catch (const mqtt::exception& exc) {
            spdlog::error("Communication: {}, then commu_enable set false.", exc.what());
            commu_enable = false;
        }
    }

    // 缓冲区初始化
    batch_size = CFG2("PointCloud", "batchSize", size_t);
    read_buffer.buffer = Eigen::MatrixXd::Zero(batch_size, 3);

    // 多面体初始化
    [[maybe_unused]] bool rslt;
    mesh = std::make_shared<TriangleMesh>();
    rslt = open3d::io::ReadTriangleMesh(CFG2("Mesh", "align", std::string), *mesh);
    assert(rslt);
    mesh->Translate({ 0, 0, -100 });
    mesh->ComputeTriangleNormals();
    mesh->ComputeVertexNormals();

    mesh_f = std::make_shared<TriangleMesh>();
    rslt = open3d::io::ReadTriangleMesh(CFG2("Mesh", "filter", std::string), *mesh_f);
    assert(rslt);
    mesh_f->Translate({ 0, 0, -100 });

    pc = std::make_shared<open3d::geometry::PointCloud>();
}
