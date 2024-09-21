#pragma once

#include <mqtt/async_client.h>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <Eigen/Core>
#include <open3d/geometry/PointCloud.h>

#include "TargetMap.h"
#include "Transform.h"

struct PcContext;

class ReconnectCallback : public virtual mqtt::callback {
    PcContext* ctx;

public:
    ReconnectCallback() {}
    ReconnectCallback(PcContext* ctx)
        : ctx(ctx)
    {
    }

    void connection_lost(const std::string& cause) override;
};

struct PcContext {
    struct ReadBuffer {
        Eigen::MatrixXd buffer;
        std::mutex mutex;
        std::condition_variable cond_var;
        std::atomic_bool flag = false;

        std::unique_lock<std::mutex> wait_and_lock(std::atomic_bool& stop_signal);
    } read_buffer;
    size_t batch_size;

    // 停止信号
    std::atomic_bool stop_signal = false;
    // 是否进入单步模式
    std::atomic_bool is_step = false;
    // 是否暂停
    std::atomic_bool is_paused = false;

    // 通信
    bool commu_enable;
    mqtt::async_client_ptr commu_client;
    mqtt::topic_ptr commu_topic_detection;

    // 可视化
    std::mutex vis_mutex;
    std::shared_ptr<open3d::geometry::PointCloud> pc;

    uint64_t pt_cnt;
    Transform trans;
    TargetMap targetMap;

    // 对齐用 mesh
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    // 过滤用 mesh
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_f;

    ReconnectCallback reconnect_callback;

    void Initialize();
};

