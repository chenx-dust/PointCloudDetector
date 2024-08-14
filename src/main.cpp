#include <fstream>
#include <atomic>
#include <csignal>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <deque>
#include <chrono>
#include <condition_variable>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <open3d/Open3D.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mqtt/async_client.h>
#include <tbb/parallel_for.h>

#include "Config.h"
#include "TargetMap.h"
#include "VoxelGrid.h"
#include "PcReceiver.h"
#include "VisualizerHelper.h"
#include "KalmanFilter.h"
#include "Clustering.h"
#include "Transform.h"

#define WAIT(CV, MUTEX, FLAG)                 \
    std::unique_lock<std::mutex> lock(MUTEX); \
    while (!FLAG && !stop_signal)             \
        CV.wait(lock);                        \
    FLAG = false;

using open3d::geometry::PointCloud;
using open3d::geometry::TriangleMesh;

YAML::Node CF_CONFIG, CF_DEFAULT;
std::thread main_thread;
//Voxel voxel[SPACE_LENGTH / VOXEL_SIZE][SPACE_WIDTH / VOXEL_SIZE][SPACE_HEIGHT / VOXEL_SIZE];
VoxelGrid voxel_;
VoxelFilter voxel_filter;
// 对齐用 mesh
std::shared_ptr<TriangleMesh> mesh;
// 过滤用 mesh
std::shared_ptr<TriangleMesh> mesh_f;
// 其他 mesh
std::shared_ptr<TriangleMesh> lidar_mesh;
std::shared_ptr<TriangleMesh> camera_mesh;
std::shared_ptr<TriangleMesh> axis_mesh;
// 读取缓冲区
Eigen::MatrixXd ReadBuffer;
std::mutex ReadBufferMutex;
std::condition_variable ReadBufferCond;
std::atomic_bool ReadBufferFlag(false);
// 转换矩阵
Transform trans;
// 待处理队列
std::deque<Eigen::Vector3d> ProcessQueue;
std::deque<size_t> SizeQueue;
// 点云
auto pc = std::make_shared<PointCloud>();
open3d::visualization::VisualizerWithKeyCallback GlobVis;
TargetMap targetMap;
std::thread reader;
std::mutex visMutex;
size_t batchSize;
size_t bufferSize, sampleSize;
uint64_t ptCnt;
double eps;
double minPointsK;
size_t minPoints;
size_t visUpdateInt, speedInt;
bool useDiff;
Eigen::Vector3d lidarPos;
// 内录
std::atomic_bool stop_signal(false);
// 过滤
size_t expected_max_points;
size_t max_density;
// 通信
bool commu_enable;
mqtt::async_client_ptr commu_client;
mqtt::topic_ptr commu_topic_detection;
// mqtt::topic_ptr commu_topic_trans;

nlohmann::json kf_dump()
{
    nlohmann::json data;
    for (const auto& target : targetMap.target_map) {
        if (target.discarded) continue;
        nlohmann::json target_json;
        target_json["id"] = target.id;
        target_json["X"] = target.kf.X;
        target_json["P"] = target.kf.P.rowwise();
        data["targets"].push_back(target_json);
    }
    return data;
}

bool key_D_debug_output(open3d::visualization::Visualizer *) {
    spdlog::info("Debug output");
    nlohmann::json data = kf_dump();
    spdlog::info("{}", data.dump());
    std::ofstream fout("debug.json");
    fout << data.dump();
    fout.close();
    return true;
}

bool key_S_save_points(open3d::visualization::Visualizer *) {
    spdlog::info("Saving points...");
    std::ofstream fout("points.csv");
    for (auto &pt : pc->points_)
        fout << pt(0) << "," << pt(1) << "," << pt(2) << std::endl;
    fout.close();
    return true;
}

std::atomic_bool is_paused(false);
std::atomic_bool is_step(false);

bool key_P_pause(open3d::visualization::Visualizer *) {
    is_paused = !is_paused;
    is_step = false;
    spdlog::info("Toogle Pause: {}", is_paused);
    return true;
}


bool key_T_single_step(open3d::visualization::Visualizer *) {
    is_step = false;
    spdlog::info("Single Step");
    return true;
}

void publish()
{
    if (!commu_enable) return;
    nlohmann::json json;
    std::vector<nlohmann::json> json_array;
    for (const auto& enemy : targetMap.target_map) {
        if (enemy.discarded) continue;
        nlohmann::json tg;
        tg["id"] = enemy.id;
        Eigen::Vector3d pt = enemy.kf.pos();
        tg["position"] = { pt(0), pt(1), pt(2) };
        Eigen::Vector3d vel = enemy.kf.velocity_rel();
        vel *= 452000.0 / batchSize;
        tg["velocity"] = { vel(0), vel(1), vel(2) };
        tg["is_predict"] = enemy.pt_num == 0;
        tg["lost_time"] = enemy.lost_time;
        tg["is_discarded"] = false;
        json_array.push_back(tg);
    }
    if (!targetMap.discarded_queue.empty()) {
        while (!targetMap.discarded_queue.empty()) {
            size_t id = targetMap.discarded_queue.front();
            targetMap.discarded_queue.pop();
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

    // MqttClient::GetInstance().publish(nullptr, "pc_detected", json_str.length(), json_str.c_str());
    commu_topic_detection->publish(json_str);

    // nlohmann::json kf_data = kf_dump();
    // std::string kf_str = kf_data.dump();
    // MqttClient::GetInstance().publish(nullptr, "kf_debug", kf_str.length(), kf_str.c_str());
}

void pub_trans()
{
    if (!commu_enable) return;
    /// 发布转换矩阵
    nlohmann::json json;
    Eigen::Matrix4d cam_trans = (trans.T * trans.E_0.inverse()).inverse();
    json["rmat"] = cam_trans(Eigen::seq(0, 2), Eigen::seq(0, 2)).rowwise();
    json["tvec_mm"] = *cam_trans(Eigen::seq(0, 2), 3).colwise().begin();
    std::string json_str = json.dump();
    // MqttClient::GetInstance().publish(nullptr, "trans", json_str.length(), json_str.c_str());
    // commu_topic_trans->publish(json_str)->wait_for(std::chrono::milliseconds(100));
    commu_client->publish(mqtt::make_message("aligned_trans", json_str, 1, true))->wait_for(std::chrono::milliseconds(100));
    spdlog::info("Communication: Published Trans");
}

void mesh_init_1() {
    [[maybe_unused]] bool rslt;
    mesh = std::make_shared<TriangleMesh>();
    rslt = open3d::io::ReadTriangleMesh(CFG2("Mesh", "align", std::string), *mesh);
    assert(rslt);
    mesh->Translate({0, 0, -100});
    mesh->ComputeTriangleNormals();
    mesh->ComputeVertexNormals();

    mesh_f = std::make_shared<TriangleMesh>();
    rslt = open3d::io::ReadTriangleMesh(CFG2("Mesh", "filter", std::string), *mesh_f);
    assert(rslt);
    mesh_f->Translate({0, 0, -100});
}

void mesh_init_2() {
    lidar_mesh = TriangleMesh::CreateCylinder(100, 500);
    camera_mesh = TriangleMesh::CreateCylinder(100, 500);
    Eigen::Matrix3d xz = Eigen::Matrix3d{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
    lidar_mesh->Rotate(xz, Eigen::Vector3d::Zero());
    lidar_mesh->Transform(trans.T);
    camera_mesh->Transform(trans.T * trans.E_0.inverse());
    lidar_mesh->PaintUniformColor({0, 0, 1});
    camera_mesh->PaintUniformColor({ 1, 0, 0 });
    axis_mesh = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1000);
}

std::shared_ptr<PointCloud> sample_pc() {
    auto pc = std::make_shared<PointCloud>();
    for (size_t i = 0; i < sampleSize; ++i) {
        WAIT(ReadBufferCond, ReadBufferMutex, ReadBufferFlag)
        pc->points_.reserve(batchSize);
        for (size_t j = 0; j < batchSize; ++j)
            pc->points_.emplace_back(ReadBuffer(j, 0), ReadBuffer(j, 1), ReadBuffer(j, 2));
    }
    return pc;
}

void stop_all(int sig) {
    spdlog::info("Stopping with signal {}", sig);
    stop_signal = true;
}

class ReconnectCallback : public virtual mqtt::callback {
public:
    void connection_lost(const std::string& cause) override
    {
        spdlog::warn("Communication: Connection lost because {}", cause);
        try {
            commu_client->reconnect()->wait();
            spdlog::info("Communication: Reconnected");
        }
        catch (const mqtt::exception& exc) {
            spdlog::error("Communication: Reconnect failed: {}", exc.what());
            commu_enable = false;
        }
    }
} reconnect_callback;

int main()
{
    Config_load();

    std::signal(SIGINT, stop_all);
    std::signal(SIGTERM, stop_all);

    mesh_init_1();

    KalmanFilter::read_params();

    batchSize = CFG2("PointCloud", "batchSize", size_t);
    bufferSize = CFG2("PointCloud", "bufferSize", size_t);
    sampleSize = CFG2("PointCloud", "sampleSize", size_t);
    ReadBuffer = Eigen::MatrixXd::Zero(batchSize, 3);

    eps = CFG3("Cluster", "basic", "eps", double);
    minPointsK = CFG3("Cluster", "basic", "minPointsK", double);
    minPoints = CFG3("Cluster", "basic", "minPoints", size_t);
    useDiff = CFG3("Cluster", "basic", "useDiff", bool);

    expected_max_points = CFG3("PointCloud", "voxelFilter", "expecteMaxPoints", size_t);

    // 通信初始化
    commu_enable = CFG2("Communication", "enable", bool);
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

    /// 开启读取线程
    spdlog::info("Starting reader thread...");
    // if (CFG3("PointCloud", "localPCD", "enable", bool))
    //     reader = PcdReaderThread(stop_signal, batchSize,
    //         CFG3("PointCloud", "localPCD", "path", std::string),
    //         CFG3("PointCloud", "localPCD", "offset", long long),
    //         ReadBuffer, ReadBufferMutex, ReadBufferCond, ReadBufferFlag,
    //         CFG3("PointCloud", "localPCD", "late", size_t));
    // else {
    //     if (CFG3("PointCloud", "replay", "enable", bool)) {
    //         reader = RecordReceiverThread(stop_signal, batchSize,
    //             CFG3("PointCloud", "replay", "path", std::string),
    //             CFG3("PointCloud", "replay", "zstdCompress", bool),
    //             ReadBuffer, ReadBufferMutex, ReadBufferCond, ReadBufferFlag);
    //     }
    //     else {
    //         reader = LivoxReceiverThread(stop_signal, batchSize, ReadBuffer, ReadBufferMutex, ReadBufferCond, ReadBufferFlag);
    //     }
    // }
    if (CFG1("RealCompetitionMode", bool)) {
        reader = LivoxReceiverThread(stop_signal, batchSize, ReadBuffer, ReadBufferMutex, ReadBufferCond, ReadBufferFlag);
    } else {
        spdlog::warn("YOU ARE IN REPLAY MODE!");
        reader = RecordReceiverThread(stop_signal, batchSize,
            CFG3("PointCloud", "replay", "path", std::string),
            CFG3("PointCloud", "replay", "zstdCompress", bool),
            ReadBuffer, ReadBufferMutex, ReadBufferCond, ReadBufferFlag);
    }
    /// 读取 100000 点, 预处理配准, 得到变换矩阵
    spdlog::info("Reading point cloud...");
    auto pc2align = sample_pc();

    /// 获取变换
    trans.initialize(pc2align, mesh, *commu_client);
    // NOTE: 注意这里会将 pc2align 变换到正确的坐标系下
    pub_trans();
    lidarPos = trans.T(Eigen::seq(0, 2), 3);
    mesh_init_2();

    /// 体素网格初始化
    voxel_.initialize();
    voxel_.occupy_by_mesh(mesh, mesh_f, lidarPos);
    voxel_.occupy_by_pc(pc2align, mesh);
    voxel_filter.initialize();

    // 可视化体素网格
    open3d::visualization::DrawGeometries({mesh, std::make_shared<open3d::geometry::VoxelGrid>(voxel_.o3d_grid), lidar_mesh}, "Voxel Grid", 1920, 1080);

    /// 初始化目标跟踪
    targetMap.initialize(voxel_);

    /// 可视化
    visUpdateInt = CFG2("Visualizer", "updateInterval", size_t);
    speedInt = CFG2("Visualizer", "speedInterval", size_t);

    GlobVis.RegisterKeyCallback(GLFW_KEY_D, key_D_debug_output);
    GlobVis.RegisterKeyCallback(GLFW_KEY_S, key_S_save_points);
    GlobVis.RegisterKeyCallback(GLFW_KEY_P, key_P_pause);
    GlobVis.RegisterKeyCallback(GLFW_KEY_T, key_T_single_step);
    GlobVis.CreateVisualizerWindow("Point Cloud Prediction", 1920, 1080);
    GlobVis.BuildUtilities();
    GlobVis.UpdateWindowTitle();
    GlobVis.AddGeometry(mesh);
    GlobVis.AddGeometry(lidar_mesh);
    GlobVis.AddGeometry(camera_mesh);
    GlobVis.AddGeometry(axis_mesh);

    auto pc_copy = std::make_shared<PointCloud>(*pc);
    GlobVis.AddGeometry(pc_copy);
    std::vector<std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox>> aabbs_copy;
    std::vector<std::shared_ptr<TriangleMesh>> spheres_copy;
    size_t tm_size = targetMap.target_map.size();
    for (const auto &target : targetMap.target_map) {
        aabbs_copy.push_back(std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(*target.aabb));
        spheres_copy.push_back(std::make_shared<TriangleMesh>(*target.mesh));
        GlobVis.AddGeometry(aabbs_copy.back());
        GlobVis.AddGeometry(spheres_copy.back());
    }

    // 主线程
    spdlog::info("Starting main thread...");
    open3d::visualization::ColorMapJet cm;
    main_thread = std::thread([&]()
    {
        // 循环
        // 可能的多线程优化
        while (reader.joinable() && !stop_signal) {
            // 获取点云数据, 单帧 1000 点
            // 读入, 滤掉点
            while (is_step && !stop_signal)
                ;
            if (is_paused) is_step = true;
            WAIT(ReadBufferCond, ReadBufferMutex, ReadBufferFlag)
            std::vector<Eigen::Vector3d> filtered;
            /// 预处理: 1. 转换坐标系 2. 过滤
            for (size_t i = 0; i < batchSize; ++i) {
                Eigen::Vector3d pt(ReadBuffer(i, 0), ReadBuffer(i, 1), ReadBuffer(i, 2));
                Eigen::Vector4d pt_ex;
                pt_ex << pt(0), pt(1), pt(2), 1;
                pt_ex = trans.T * pt_ex;
                Eigen::Vector3d pt_t = pt_ex.head<3>();
                if (!voxel_.is_occupied(pt_t))
                    filtered.emplace_back(std::move(pt_t));
            }
            lock.unlock();
            ProcessQueue.insert(ProcessQueue.end(), filtered.begin(), filtered.end());
            SizeQueue.emplace_back(filtered.size());
            if (SizeQueue.size() > bufferSize)
                ProcessQueue.erase(ProcessQueue.begin(), ProcessQueue.begin() + SizeQueue.front()),
                SizeQueue.pop_front();
            if (filtered.empty())
                continue;
            std::lock_guard<std::mutex> lock_(visMutex);
            ptCnt += batchSize;
            pc->points_.clear();
            pc->points_.insert(pc->points_.end(), ProcessQueue.begin(), ProcessQueue.end());
//            spdlog::info("Original point cloud size: {}", pc->points_.size());
            if (pc->points_.size() > expected_max_points)
                pc->points_ = voxel_filter.filter(pc->points_, double(expected_max_points) / pc->points_.size());
            // pc->points_ = TargetMap::voxel_filter_d(pc->points_, f_voxel_size, max_density);
//            spdlog::info("Point cloud size: {}", pc->points_.size());
            /// 聚类
            std::vector<std::shared_ptr<PointCloud> > pcs;
            PointCloud pc_noise;
            std::vector<int> labels;
            std::vector<open3d::geometry::AxisAlignedBoundingBox> aabbs;
            std::vector<size_t> pt_num;
            std::vector<Eigen::Vector3d> grav;
            if (useDiff)
                TargetMap::cluster_d(*pc, lidarPos, eps, minPointsK, labels, pcs, pc_noise, aabbs, pt_num, grav);
            else
                TargetMap::cluster(*pc, eps, minPoints, labels, pcs, pc_noise, aabbs, pt_num, grav);
            std::vector<int> id_map = targetMap.update(aabbs, pt_num, grav, *pc);
            // 发布识别结果
            publish();
            // 分配颜色
            pc->colors_.clear();
            for (const auto l: labels)
                if (l < 0)
                    pc->colors_.push_back(Eigen::Vector3d::Ones());
                else if (id_map[l] == TM_NEED_SEPERATE)
                    pc->colors_.push_back({1.0, 0.7, 0.9});
                else
                    pc->colors_.push_back(cm.GetColor(id_map[l] % 11 / 10.0));
        }
    });
    while (reader.joinable() && !stop_signal) {
        static size_t report_cnt = 0;
        {
            std::lock_guard<std::mutex> lock_(visMutex);
            *pc_copy = *pc;
            for (size_t i = 0; i < tm_size; ++i) {
                *aabbs_copy[i] = *targetMap.target_map[i].aabb;
                *spheres_copy[i] = *targetMap.target_map[i].mesh;
            }
            if (report_cnt >= speedInt) {
                spdlog::info("process speed: {} pt/s, rate: {}",
                            ptCnt * 1000 / (visUpdateInt * speedInt),
                            ptCnt * 1000 / (visUpdateInt * speedInt * 452000.0));
                report_cnt = 0;
                ptCnt = 0;
            }
        }
        report_cnt++;
        GlobVis.PollEvents();
        GlobVis.UpdateGeometry();
        GlobVis.UpdateRender();
        std::this_thread::sleep_for(std::chrono::milliseconds(visUpdateInt));
    }
    GlobVis.DestroyVisualizerWindow();
    if (main_thread.joinable())
        main_thread.join();
    spdlog::info("MainThread: Stopped");
    if (reader.joinable())
        reader.join();
    spdlog::info("Reader: Stopped");
    if (commu_enable)
        commu_client->disconnect()->wait();
    spdlog::info("Communication: Stopped");
    return 0;
}
