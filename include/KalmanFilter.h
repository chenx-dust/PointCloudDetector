
#pragma once

#include <Eigen/Core>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;


class KalmanFilter {
// protected:
public:
    Vector6d X, X_; // X 状态变量, X_ 先验状态变量
    Matrix6d P, P_; // P 协方差矩阵, P_ 先验协方差矩阵
    size_t lost_time;
    void predict();

public:
    static void read_params();
    /// X0 初始位置
    void init(const Eigen::Vector3d& X0, size_t p_times);
    /// 无观测, 只预测
    void update();
    /// Z 观测位置
    void update(const Eigen::Vector3d& Z);

    inline Eigen::Vector3d pos() const {
        /// @brief 返回位置
        return X(Eigen::seq(0, 2));
    }
    inline Eigen::Vector3d velocity_rel() const {
        /// @brief 返回相对速度
        return X(Eigen::seq(3, 5));
    }
    inline double possibility(Eigen::Vector3d pt) const {
        /// @brief 计算概率密度
        Eigen::Matrix3d cov = P(Eigen::seq(0, 2), Eigen::seq(0, 2));
        Eigen::Vector3d d = pt - pos();
        return 1 / sqrt(pow(2 * M_PI, 3) * cov.determinant()) * exp(-0.5 * d.transpose() * cov.inverse() * d);
    }
};
