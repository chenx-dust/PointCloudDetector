#include "PcReceiver.h"
#include "Config.h"

void PcReceiver::Initialize(std::shared_ptr<PcContext> ctx_)
{
    ctx = ctx_;
    ctx->read_buffer.flag = false;
}

void PcReceiver::Loop()
{
    if (CFG1("RealCompetitionMode", bool)) {
        LivoxReceiverLoop();
    } else {
        spdlog::warn("YOU ARE IN REPLAY MODE!");
        RecordReceiverLoop(
            CFG3("PointCloud", "replay", "path", std::string),
            CFG3("PointCloud", "replay", "zstdCompress", bool));
    }
}
