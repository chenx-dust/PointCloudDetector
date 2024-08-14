#pragma once

#include <Eigen/Core>
#include <fstream>
#include <spdlog/spdlog.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "Recorder.h"

std::thread PcdReaderThread(std::atomic_bool& stop_sig, size_t batch_size, const std::string& filename, std::streamoff offset, Eigen::MatrixXd& points, std::mutex& mutex, std::condition_variable& cv, std::atomic_bool& flag, size_t late);
std::thread LivoxReceiverThread(std::atomic_bool& stop_sig, size_t batch_size, Eigen::MatrixXd& points, std::mutex& mutex, std::condition_variable& cv, std::atomic_bool& flag);
std::thread RecordReceiverThread(std::atomic_bool& stop_sig, size_t batch_size, const std::string& record_filename, bool use_zstd, Eigen::MatrixXd& points, std::mutex& mutex, std::condition_variable& cv, std::atomic_bool& flag);