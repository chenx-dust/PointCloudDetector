#pragma once

#include <Eigen/Core>
#include <open3d/Open3D.h>
#include <spdlog/spdlog.h>
#include <mqtt/async_client.h>


class Transform {
public:
    Eigen::Matrix4d T, E_0;

    void initialize(std::shared_ptr<open3d::geometry::PointCloud> pc2align,
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh_, mqtt::async_client& client);

    // 下面是实用函数
    static inline Eigen::Matrix4d get_transform(const Eigen::Matrix3d &rmat, const Eigen::Vector3d &tvec) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rmat;
        T.block<3, 1>(0, 3) = tvec;
        return T;
    }

    static inline void print_transform(const Eigen::Matrix4d &trans) {
//        spdlog::info("\n\t[[{},\t{},\t{},\t{}\t], \n\t[{},\t{},\t{},\t{}\t], \n\t[{},\t{},\t{},\t{}\t], \n\t[{},\t{},\t{},\t{}\t]]",
//                    trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3),
//                    trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3),
//                    trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3),
//                    trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
        spdlog::info("rmat: [[{}, {}, {}],\n [{}, {}, {}],\n [{}, {}, {}]]",
                     trans(0, 0), trans(0, 1), trans(0, 2),
                     trans(1, 0), trans(1, 1), trans(1, 2),
                     trans(2, 0), trans(2, 1), trans(2, 2));
        spdlog::info("tvec(mm): [{}, {}, {}]", trans(0, 3), trans(1, 3), trans(2, 3));
        spdlog::info("tvec(m): [{}, {}, {}]", trans(0, 3) / 1000.0, trans(1, 3) / 1000.0, trans(2, 3) / 1000.0);
    }
};
