#include "PcReceiver.h"

#include <Eigen/Eigen>
#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/endian/arithmetic.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <chrono>
#include <fmt/format.h>
#include <future>
#include <iostream>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>

using namespace boost::endian;

#pragma pack(push, 1)
struct header {
    little_uint8_t version;
    little_uint16_t length;
    little_uint16_t time_interval;
    little_uint16_t dot_num;
    little_uint16_t udp_cnt;
    little_uint8_t frame_cnt;
    little_uint8_t data_type;
    little_uint8_t time_type;
    little_uint8_t pack_info;
    little_uint8_t _padding[11];
    little_uint32_t crc32;
    little_uint64_t timestamp;
};
struct pcd1 {
    little_int32_t x;
    little_int32_t y;
    little_int32_t z;
    little_uint8_t reflectivity;
    little_uint8_t tag;
};
using pcd1_span = std::array<struct pcd1, 96>;
#pragma pack(pop)

void PcReceiver::RecordReceiverLoop(const std::string &record_filename, bool use_zstd)
{
    const size_t times = ctx->batch_size / 96;
    size_t idx = 0;
    boost::iostreams::filtering_istream in;
    // TODO: 区分
    if (use_zstd)
        in.push(boost::iostreams::zstd_decompressor());
    auto file = boost::iostreams::file_source(record_filename, std::ios_base::in | std::ios_base::binary);
    in.push(file);
    spdlog::info("Record: Read from {}", record_filename);
    std::optional<std::uint64_t> last_timestamp; // 用于计算要 sleep 多久才能匹配
    while (!ctx->stop_signal && !in.eof()) {
        std::array<unsigned char, 1380> recv_buf {};

        in.read(reinterpret_cast<char*>(recv_buf.data()), recv_buf.size());

        auto header = reinterpret_cast<struct header*>(recv_buf.data());
        switch (header->time_type) {
        case 0:
            while (false)
                ; // fmt::print("time_type: 开机时间\n");
            break;
        case 1:
            while (false)
                ; // fmt::print("time_type: 时钟源时间\n");
            break;
        default:
            while (false)
                ; // fmt::print("time_type: 异常({})\n", header->time_type.value());
        }
        //        while(false);// fmt::print("time_type: {}\n", header->time_type.value());

        switch (header->pack_info.value() & 0x03) {
        case 0:
            while (false)
                ; // fmt::print("pack_info: 整包可信\n");
            break;
        case 1:
            while (false)
                ; // fmt::print("pack_info: 整包不可信\n");
            break;
        case 2:
            while (false)
                ; // fmt::print("pack_info: 非0点可信\n");
            break;
        default:
            while (false)
                ; // fmt::print("pack_info: 异常({})\n", header->pack_info.value() & 0x03);
        }

        switch (header->data_type) {
        case 0:
            while (false)
                ; // fmt::print("data_type: IMU Data\n");
            break;
        case 1:
            while (false)
                ; // fmt::print("data_type: Point Cloud Data1\n");
            {
                std::unique_lock<std::mutex> lock(ctx->read_buffer.mutex);
                auto data = reinterpret_cast<pcd1_span*>(recv_buf.data() + sizeof(struct header));
                //                        Eigen::Matrix<uint32_t, 96, 3> position_mm;
                //                        Eigen::Matrix<uint8_t, 96, 1> reflectivity;
                //                        Eigen::Matrix<uint8_t, 96, 1> tag;

                for (int i = 0; i < 96; ++i) {
                    //                            position_mm(i, 0) = data->at(i).x.value();
                    //                            position_mm(i, 1) = data->at(i).y.value();
                    //                            position_mm(i, 2) = data->at(i).z.value();
                    //                            reflectivity(i, 0) = data->at(i).reflectivity.value();
                    //                            tag(i, 0) = data->at(i).tag.value();
                    ctx->read_buffer.buffer(idx * 96 + i, 0) = data->at(i).x.value();
                    ctx->read_buffer.buffer(idx * 96 + i, 1) = data->at(i).y.value();
                    ctx->read_buffer.buffer(idx * 96 + i, 2) = data->at(i).z.value();
                }

                //                        std::cout << "position_mm:\n" << position_mm << std::endl;
                ++idx;
                if (idx >= times) {
                    idx = 0;
                    ctx->read_buffer.flag = true;
                    ctx->read_buffer.cond_var.notify_one();
                    if (last_timestamp) {
                        const auto interval = std::chrono::nanoseconds(header->timestamp - *last_timestamp);
                        last_timestamp = header->timestamp;
                        std::this_thread::sleep_for(interval);
                    } else {
                        last_timestamp = header->timestamp;
                    }
                    //                            spdlog::info("Livox read finished!");
                }

                //                    for (auto &i: *data) {
                //                        while(false);// fmt::print("x: {}, y: {}, z: {}, reflectivity: {}, tag: {}\n",
                //                                   i.x.value(), i.y.value(), i.z.value(), i.reflectivity.value(), i.tag.value());
                //                    }
            }
            break;
        case 2:
            while (false)
                ; // fmt::print("data_type: Point Cloud Data2\n");
            break;
        default:
            while (false)
                ; // fmt::print("data_type: 异常({})\n", header->time_type.value());
        }
    }
    ctx->stop_signal = true; // 提示其他线程停止
    spdlog::info("Record read finished!");
    ctx->read_buffer.cond_var.notify_all();
}
