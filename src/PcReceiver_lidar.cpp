#include "PcReceiver.h"
#include <Eigen/Eigen>
#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/endian/arithmetic.hpp>
#include <spdlog/spdlog.h>
#include <thread>
#include <future>

using boost::asio::ip::udp;
using namespace boost::endian;

constexpr size_t msg_size = 1380;

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

void PcReceiver::LivoxReceiverLoop()
{
    Recorder recorder;
    recorder.initialize();
    const size_t times = ctx->batch_size / 96;
    size_t idx = 0;
    boost::asio::io_context io_context;
    bool is_first = true;
    udp::socket socket(io_context, udp::endpoint(udp::v4(), 57000));

    while(!ctx->stop_signal) {
        std::array<unsigned char, msg_size> recv_buf {};
        boost::system::error_code error;
        std::size_t recv_length = 0;
        // https://github.com/boostorg/asio/blob/develop/example/cpp11/timeouts/blocking_udp_client.cpp

        socket.async_receive(boost::asio::buffer(recv_buf),
            [&](const boost::system::error_code& error_, std::size_t length_) {
                error = error_;
                recv_length = length_;
            });

        io_context.restart();
        io_context.run_for(std::chrono::seconds(1));
        if (!io_context.stopped()) {
            socket.cancel();
            io_context.run();
        }

        if (recv_length != msg_size) {
            spdlog::error("Lidar: Wrong receive length: {}, error: {} (Maybe timeout?)", recv_length,error.message());
            continue;
        }

        if (error && error != boost::asio::error::message_size) {
            spdlog::error("Lidar: Receiver error: {}", error.message());
            continue;
        }

        if (is_first) {
            spdlog::info("Lidar: Receiver works well");
            is_first = false;
        }

        // 内录
        recorder.write(recv_buf);

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
        case 0:     // 整包可信
            break;
        case 1:     // 整包不可信
            continue;
        case 2:     // 非0点可信
            break;
        default:
            spdlog::error("Lidar: pack_info: 异常({})\n", header->pack_info.value() & 0x03);
        }

        switch (header->data_type) {
        case 0:     // IMU Data
            break;
        case 1:
            {
            std::unique_lock<std::mutex> lock(ctx->read_buffer.mutex);
            auto data = reinterpret_cast<pcd1_span*>(recv_buf.data() + sizeof(struct header));

            for (int i = 0; i < 96; ++i) {
                ctx->read_buffer.buffer(idx * 96 + i, 0) = data->at(i).x.value();
                ctx->read_buffer.buffer(idx * 96 + i, 1) = data->at(i).y.value();
                ctx->read_buffer.buffer(idx * 96 + i, 2) = data->at(i).z.value();
                }

                ++idx;
                if (idx >= times) {
                    idx = 0;
                    ctx->read_buffer.flag = true;
                    ctx->read_buffer.cond_var.notify_one();
                }
            }
            break;
        case 2:
            break;
        default:
            spdlog::error("Lidar: data_type: 异常({})\n", header->time_type.value());
        }
    }
    ctx->read_buffer.cond_var.notify_all();
    recorder.stop();
    spdlog::info("Lidar: LivoxReceiverThread & Recorder stopped");
}
