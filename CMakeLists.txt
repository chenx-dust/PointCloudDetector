cmake_minimum_required(VERSION 3.22)
project(PointCloudDetection)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O3 -Wall -Wextra)

find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost COMPONENTS date_time filesystem iostreams REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(TBB REQUIRED)
find_package(PahoMqttCpp REQUIRED)

add_executable(PointCloudDetection)

file(GLOB_RECURSE SRC_FILES src/*.cpp)
target_sources(PointCloudDetection PRIVATE ${SRC_FILES})

file(COPY resource DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
configure_file(config.yaml ${CMAKE_CURRENT_BINARY_DIR}/config.yaml COPYONLY)
configure_file(default.yaml ${CMAKE_CURRENT_BINARY_DIR}/default.yaml COPYONLY)

target_include_directories(PointCloudDetection PRIVATE include)

target_link_libraries(PointCloudDetection PRIVATE
        Open3D::Open3D
        spdlog::spdlog_header_only
        yaml-cpp
        Boost::boost
        Boost::date_time
        Boost::filesystem
        Boost::iostreams
        nlohmann_json::nlohmann_json
        PahoMqttCpp::paho-mqttpp3
        TBB::tbb
        Eigen3::Eigen
        )

