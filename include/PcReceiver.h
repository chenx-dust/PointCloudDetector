#pragma once

#include <Eigen/Core>
#include <fstream>
#include <spdlog/spdlog.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "Recorder.h"
#include "PcContext.h"

class PcReceiver {
    std::shared_ptr<PcContext> ctx;
    void LivoxReceiverLoop();
    void RecordReceiverLoop(const std::string& record_filename, bool use_zstd);

public:
    void Initialize(std::shared_ptr<PcContext> ctx);
    void Loop();
};