#include "Recorder.h"
#include "Config.h"

#include <fstream>
#include <array>
#include <mutex>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include <spdlog/spdlog.h>

void Recorder::initialize() {
    is_enabled = CFG3("PointCloud", "recorder", "enable", bool);
    if (is_enabled) {
        boost::filesystem::path d_path = CFG3("PointCloud", "recorder", "directory", std::string);
        bool do_zstd_compress = CFG3("PointCloud", "recorder", "zstdCompress", bool);
        if (!boost::filesystem::exists(d_path)) {
            spdlog::info("Created directory: {}", d_path.string());
            boost::filesystem::create_directories(d_path);
        }
        std::string time_str = boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
        std::string filename_str = time_str + ".lidardump";
        if (do_zstd_compress)
            filename_str += ".zst";
        boost::filesystem::path path = d_path / filename_str;
        // file = std::fstream(path_str, std::ios::out | std::ios::binary);
        if (do_zstd_compress)
            out.push(boost::iostreams::zstd_compressor());
        file = boost::iostreams::file_sink(path.string(), std::ios::out | std::ios::binary);
        out.push(file.get());
        spdlog::info("Recorder: file opened: {}", path.string());
    }
}

void Recorder::write(const std::array<unsigned char, 1380> &recv_buf) {
    if (is_enabled) {
        if (!file.is_initialized() && !file.get().is_open()) {
            spdlog::error("Recorder: file is not open");
            is_enabled = false;
            return;
        }
        out.write(reinterpret_cast<const char*>(recv_buf.data()), recv_buf.size());
        out.flush();
    }
}

void Recorder::stop() {
    if (is_enabled) {
        is_enabled = false;
        out.flush();
        spdlog::info("Recorder: out.flush() finished");
        out.set_auto_close(true);
        out.reset();
        spdlog::info("Recorder: out.reset() finished, file closed: {}", !file.get().is_open());
    }
}
