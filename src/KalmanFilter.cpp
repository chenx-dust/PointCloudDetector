
#include "KalmanFilter.h"
#include "Config.h"
#include <Eigen/Dense>

double Kf_speed_limit, cov_factor;
size_t predict_stop;

Matrix6d Kf_A, Kf_Q, Kf_R;

void KalmanFilter::read_params() {
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            Kf_Q(i, j) = CFG4("KalmanFilter", "Q", i, j, double);
            Kf_R(i, j) = CFG4("KalmanFilter", "R", i, j, double);
        }
    double decay_rate = CFG2("KalmanFilter", "decayRate", double);
    double z_decay_rate = CFG2("KalmanFilter", "zDecayRate", double);
    Kf_A = Matrix6d {
        {1, 0, 0, 1, 0, 0},
        {0, 1, 0, 0, 1, 0},
        {0, 0, 1, 0, 0, 1},
        {0, 0, 0, 1 - decay_rate, 0, 0},
        {0, 0, 0, 0, 1 - decay_rate, 0},
        {0, 0, 0, 0, 0, 1 - z_decay_rate}
    };
    Kf_speed_limit = CFG2("KalmanFilter", "speedLimit", double);
    cov_factor = CFG2("KalmanFilter", "covFactor", double);
    predict_stop = CFG2("KalmanFilter", "predictStop", size_t);
}

void KalmanFilter::init(const Eigen::Vector3d &X0, size_t p_times) {
    X << X0[0], X0[1], X0[2], 0, 0, 0;
    P = Kf_Q * p_times;
    X_ = X;
    P_ = P;
    lost_time = 0;
}

void KalmanFilter::predict() {
    X_ = Kf_A * X;
    X_(Eigen::seq(3, 5)) = X_(Eigen::seq(3, 5))
                                        .cwiseMax(-Kf_speed_limit)
                                        .cwiseMin(Kf_speed_limit);
    if (lost_time < predict_stop) {
        P_ = Kf_A * P * Kf_A.transpose() + Kf_Q;
        P_(Eigen::seq(0, 2), Eigen::seq(0, 2)) += X_(Eigen::seq(3, 5)) * X_(Eigen::seq(3, 5)).transpose() * cov_factor;
    }
}

void KalmanFilter::update() {
    predict();
    X = X_;
    P = P_;
    lost_time++;
}

void KalmanFilter::update(const Eigen::Vector3d &Z) {
    predict();
    // 利用坐标作差得到观测速度
    Vector6d Z_v;
    Z_v << Z, (Z - X(Eigen::seq(0, 2))) / (double)(lost_time + 1);
    // 计算卡尔曼增益
    Matrix6d K = P_ * Eigen::Inverse(P_ + Kf_R);
    // 更新状态变量
    Vector6d d = (Z_v - X_);
    X = X_ + K * d;
    // 更新协方差矩阵
    // P = (Matrix6d::Identity() - K) * P_ + d * d.transpose() * cov_factor / (lost_time + 1);
    P = (Matrix6d::Identity() - K) * P_;
    // P(Eigen::seq(0, 2), Eigen::seq(0, 2)) += X(Eigen::seq(3, 5)) * X(Eigen::seq(3, 5)).transpose() * cov_factor;
    lost_time = 0;
}

