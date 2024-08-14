#include "PcReceiver.h"
#include "fast_float/fast_float.h"

std::thread PcdReaderThread(std::atomic_bool& stop_sig, size_t batch_size, const std::string& filename, std::streamoff offset,
                            Eigen::MatrixXd& points, std::mutex& mutex, std::condition_variable& cv, std::atomic_bool& flag, size_t late)
{
    flag = false;
    std::thread thread([&, batch_size, offset, filename, late]() {
        spdlog::info("From file: {} with offset: {}", filename, offset);
        std::fstream file(filename, std::ios::in);
        std::string buf;
        file.seekg(offset);
        while (file.get() != '\n')
            ;
        while (file.is_open() && !file.eof() && !stop_sig) {
            std::unique_lock<std::mutex> lock(mutex);
            for (size_t i = 0; i < batch_size; i++) {
                std::getline(file, buf);
                if (buf.empty()) break;
                double d;
                auto answer = fast_float::from_chars(buf.data(), buf.data() + buf.size(), d);
//                if (answer.ec != std::errc()) {
//                    spdlog::error("Error reading point: {}", buf);
//                    spdlog::error("Error code: {}", std::make_error_code(answer.ec).message());
//                    spdlog::error("Error position: {}", answer.ptr - buf.data());
//                    break;
//                }
                points(i, 0) = d * 1000;
                answer = fast_float::from_chars(answer.ptr + 1, buf.data() + buf.size(), d);
                points(i, 1) = d * 1000;
                fast_float::from_chars(answer.ptr + 1, buf.data() + buf.size(), d);
                points(i, 2) = d * 1000;
                //                spdlog::info("Read point: {}, {}, {}", points(i, 0), points(i, 1), points(i, 2));
            }
            flag = true;
            lock.unlock();
            cv.notify_all();
            while (flag)
                ;
            // 小等一会
            std::this_thread::sleep_for(std::chrono::milliseconds(late));
        }
        spdlog::info("Read finished.");
        stop_sig = true;
        cv.notify_all();
    });
    return thread;
}