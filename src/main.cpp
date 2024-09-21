#include <csignal>

#include "Config.h"
#include "PcContext.h"
#include "PcDetector.h"
#include "PcReceiver.h"
#include "PcVisualizer.h"

YAML::Node CF_CONFIG, CF_DEFAULT;
std::shared_ptr<PcContext> ctx;
std::thread tReceiver, tDetector;
PcReceiver receiver;
PcDetector detector;
PcVisualizer visualizer;

void stop_all(int sig) {
    spdlog::info("Stopping with signal {}", sig);
    ctx->stop_signal = true;
}

int main()
{
    Config_load();

    std::signal(SIGINT, stop_all);
    std::signal(SIGTERM, stop_all);

    // 初始化
    KalmanFilter::read_params();
    ctx = std::make_shared<PcContext>();
    ctx->Initialize();

    // 点云接收线程
    receiver.Initialize(ctx);
    spdlog::info("tReceiver: Starting");
    tReceiver = std::thread(std::bind(&PcReceiver::Loop, receiver));

    // 点云处理线程
    detector.Initialize(ctx);
    visualizer.Initialize(ctx);
    spdlog::info("tDetector: Starting");
    tDetector = std::thread(std::bind(&PcDetector::Loop, detector));
    visualizer.Loop();

    if (tDetector.joinable())
        tDetector.join();
    spdlog::info("tDetector: Stopped");
    if (tReceiver.joinable())
        tReceiver.join();
    spdlog::info("tReceiver: Stopped");
    if (ctx->commu_enable)
        ctx->commu_client->disconnect()->wait();
    spdlog::info("Communication: Stopped");
    return 0;
}
