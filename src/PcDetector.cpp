#include "PcDetector.h"
#include <nlohmann/json.hpp>

using open3d::geometry::PointCloud;

void PcDetector::Initialize(std::shared_ptr<PcContext> ctx_)
{
    ctx = ctx_;
    buffer_size = CFG2("PointCloud", "bufferSize", size_t);
    sample_size = CFG2("PointCloud", "sampleSize", size_t);

    eps = CFG3("Cluster", "basic", "eps", double);
    min_points_k = CFG3("Cluster", "basic", "minPointsK", double);
    min_points = CFG3("Cluster", "basic", "minPoints", size_t);
    use_diff = CFG3("Cluster", "basic", "useDiff", bool);

    expected_max_points = CFG3("PointCloud", "voxelFilter", "expecteMaxPoints", size_t); /// 体素网格初始化

    /// 读取 100000 点, 预处理配准, 得到变换矩阵
    spdlog::info("Reading point cloud...");
    auto pc2align = sample_pc();

    /// 获取变换
    ctx->trans.initialize(pc2align, ctx->mesh, *ctx->commu_client);
    // NOTE: 注意这里会将 pc2align 变换到正确的坐标系下
    pub_trans();
    lidar_pos = ctx->trans.T(Eigen::seq(0, 2), 3);

    voxel_grid.initialize();
    voxel_grid.occupy_by_mesh(ctx->mesh, ctx->mesh_f, lidar_pos);
    voxel_grid.occupy_by_pc(pc2align, ctx->mesh);
    voxel_filter.initialize();

    // 可视化体素网格
    open3d::visualization::DrawGeometries({ ctx->mesh, std::make_shared<open3d::geometry::VoxelGrid>(voxel_grid.o3d_grid) }, "Voxel Grid", 1920, 1080);

    /// 初始化目标跟踪
    ctx->targetMap.initialize(voxel_grid);
}

void PcDetector::Loop()
{
    while (!ctx->stop_signal) {
        // 获取点云数据, 单帧 1000 点
        // 读入, 滤掉点
        while (ctx->is_step && !ctx->stop_signal)
            ;
        if (ctx->is_paused)
            ctx->is_step = true;
        auto lock = ctx->read_buffer.wait_and_lock(ctx->stop_signal);
        std::vector<Eigen::Vector3d> filtered;
        /// 预处理: 1. 转换坐标系 2. 过滤
        for (size_t i = 0; i < ctx->batch_size; ++i) {
            Eigen::Vector4d pt_ex;
            pt_ex << ctx->read_buffer.buffer(i, 0), ctx->read_buffer.buffer(i, 1), ctx->read_buffer.buffer(i, 2), 1;
            pt_ex = ctx->trans.T * pt_ex;
            Eigen::Vector3d pt_t = pt_ex.head<3>();
            if (!voxel_grid.is_occupied(pt_t))
                filtered.emplace_back(std::move(pt_t));
        }
        lock.unlock();
        process_queue.insert(process_queue.end(), filtered.begin(), filtered.end());
        size_queue.emplace_back(filtered.size());
        if (size_queue.size() > buffer_size)
            process_queue.erase(process_queue.begin(), process_queue.begin() + size_queue.front()),
                size_queue.pop_front();
        if (filtered.empty())
            continue;
        std::lock_guard<std::mutex> lock_(ctx->vis_mutex);
        ctx->pt_cnt += ctx->batch_size;
        ctx->pc->points_.clear();
        ctx->pc->points_.insert(ctx->pc->points_.end(), process_queue.begin(), process_queue.end());
        //            spdlog::info("Original point cloud size: {}", pc->points_.size());
        if (ctx->pc->points_.size() > expected_max_points)
            ctx->pc->points_ = voxel_filter.filter(ctx->pc->points_, double(expected_max_points) / ctx->pc->points_.size());
        // pc->points_ = TargetMap::voxel_filter_d(pc->points_, f_voxel_size, max_density);
        //            spdlog::info("Point cloud size: {}", pc->points_.size());
        /// 聚类
        std::vector<std::shared_ptr<PointCloud>> pcs;
        PointCloud pc_noise;
        std::vector<int> labels;
        std::vector<open3d::geometry::AxisAlignedBoundingBox> aabbs;
        std::vector<size_t> pt_num;
        std::vector<Eigen::Vector3d> grav;
        if (use_diff)
            TargetMap::cluster_d(*ctx->pc, lidar_pos, eps, min_points_k, labels, pcs, pc_noise, aabbs, pt_num, grav);
        else
            TargetMap::cluster(*ctx->pc, eps, min_points, labels, pcs, pc_noise, aabbs, pt_num, grav);
        std::vector<int> id_map = ctx->targetMap.update(aabbs, pt_num, grav, *ctx->pc);
        // 发布识别结果
        pub_targets();
        // 分配颜色
        ctx->pc->colors_.clear();
        open3d::visualization::ColorMapJet cm;

        for (const auto l : labels)
            if (l < 0)
                ctx->pc->colors_.push_back(Eigen::Vector3d::Ones());
            else if (id_map[l] == TM_NEED_SEPERATE)
                ctx->pc->colors_.push_back({ 1.0, 0.7, 0.9 });
            else
                ctx->pc->colors_.push_back(cm.GetColor(id_map[l] % 11 / 10.0));
    }
}

void PcDetector::pub_targets()
{
    if (!ctx->commu_enable)
        return;
    nlohmann::json json;
    std::vector<nlohmann::json> json_array;
    for (const auto& enemy : ctx->targetMap.target_map) {
        if (enemy.discarded)
            continue;
        nlohmann::json tg;
        tg["id"] = enemy.id;
        Eigen::Vector3d pt = enemy.kf.pos();
        tg["position"] = { pt(0), pt(1), pt(2) };
        Eigen::Vector3d vel = enemy.kf.velocity_rel();
        vel *= 452000.0 / ctx->batch_size;
        tg["velocity"] = { vel(0), vel(1), vel(2) };
        tg["is_predict"] = enemy.pt_num == 0;
        tg["lost_time"] = enemy.lost_time;
        tg["is_discarded"] = false;
        json_array.push_back(tg);
    }
    if (!ctx->targetMap.discarded_queue.empty()) {
        while (!ctx->targetMap.discarded_queue.empty()) {
            size_t id = ctx->targetMap.discarded_queue.front();
            ctx->targetMap.discarded_queue.pop();
            nlohmann::json tg;
            tg["id"] = id;
            tg["position"] = { -1., -1., -1. };
            tg["velocity"] = { -1., -1., -1. };
            tg["is_predict"] = true;
            tg["lost_time"] = 0xffff;
            tg["is_discarded"] = true;
            json_array.push_back(tg);
        }
    }
    json["enemies"] = json_array;
    std::string json_str = json.dump();

    ctx->commu_topic_detection->publish(json_str);
}

std::shared_ptr<PointCloud> PcDetector::sample_pc()
{
    auto pc = std::make_shared<PointCloud>();
    for (size_t i = 0; i < sample_size; ++i) {
        ctx->read_buffer.wait_and_lock(ctx->stop_signal);
        pc->points_.reserve(ctx->batch_size);
        for (size_t j = 0; j < ctx->batch_size; ++j)
            pc->points_.emplace_back(ctx->read_buffer.buffer(j, 0), ctx->read_buffer.buffer(j, 1), ctx->read_buffer.buffer(j, 2));
    }
    return pc;
}

void PcDetector::pub_trans()
{
    if (!ctx->commu_enable)
        return;
    /// 发布转换矩阵
    nlohmann::json json;
    Eigen::Matrix4d cam_trans = (ctx->trans.T * ctx->trans.E_0.inverse()).inverse();
    json["rmat"] = cam_trans(Eigen::seq(0, 2), Eigen::seq(0, 2)).rowwise();
    json["tvec_mm"] = *cam_trans(Eigen::seq(0, 2), 3).colwise().begin();
    std::string json_str = json.dump();
    // MqttClient::GetInstance().publish(nullptr, "trans", json_str.length(), json_str.c_str());
    // commu_topic_trans->publish(json_str)->wait_for(std::chrono::milliseconds(100));
    ctx->commu_client->publish(mqtt::make_message("aligned_trans", json_str, 1, true))->wait_for(std::chrono::milliseconds(100));
    spdlog::info("Communication: Published Trans");
}
