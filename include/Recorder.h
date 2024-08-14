#pragma once

#include <array>
#include <fstream>
#include <mutex>
#include <boost/optional.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>

class Recorder {
private:
    bool is_enabled = false;
    // std::fstream file;
    boost::optional<boost::iostreams::file_sink> file;
    boost::iostreams::filtering_ostream out;
public:
    void initialize();
    void write(const std::array<unsigned char, 1380> &recv_buf);
    void stop();
};
