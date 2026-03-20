#include "KFfilter.h"

namespace mpcc {

KFfilter::KFfilter() {
    X_.setZero();
    P_.setIdentity();
    
    // 观测矩阵 H: 只观测位置和速度 [I_3x3, 0_3x3, 0_3x3; 0_3x3, I_3x3, 0_3x3]
    H_.setZero();
    H_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    
    // 观测噪声 R: 0.5Hz 的数据虽然慢，但假设雷达/通信给的值相对准确
    R_.setIdentity();
    R_.block<3, 3>(0, 0) *= 1.0;  // 位置噪声方差
    R_.block<3, 3>(3, 3) *= 2.0;  // 速度噪声方差
    
    // 过程噪声 Q: 允许加速度剧烈变化，以此来追踪高机动目标
    Q_.setIdentity();
    Q_.block<3, 3>(0, 0) *= 0.1;   // 位置预测极度信任物理学
    Q_.block<3, 3>(3, 3) *= 0.1;   // 速度预测信任物理学
    Q_.block<3, 3>(6, 6) *= 50.0;  // 加速度过程噪声极大，让滤波器积极调整加速度来补偿位置/速度残差
}

void KFfilter::init(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0) {
    X_.setZero();
    X_.segment<3>(0) = p0;
    X_.segment<3>(3) = v0;
    P_ = Eigen::Matrix<double, 9, 9>::Identity() * 10.0; // 初始协方差较大
}

void KFfilter::predict(double dt) {
    // 构建状态转移矩阵 A (带时间步长 dt)
    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
    A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    A.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * (0.5 * dt * dt);
    A.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
    
    // X = A * X
    X_ = A * X_;
    // P = A * P * A^T + Q
    P_ = A * P_ * A.transpose() + Q_;
}

void KFfilter::update(const Eigen::Vector3d& p_meas, const Eigen::Vector3d& v_meas) {
    Eigen::Matrix<double, 6, 1> Z;
    Z.segment<3>(0) = p_meas;
    Z.segment<3>(3) = v_meas;
    
    // 卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
    Eigen::Matrix<double, 6, 6> S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double, 9, 6> K = P_ * H_.transpose() * S.inverse();
    
    // 状态更新 X = X + K * (Z - H * X)
    X_ = X_ + K * (Z - H_ * X_);
    
    // 协方差更新 P = (I - K * H) * P
    Eigen::Matrix<double, 9, 9> I = Eigen::Matrix<double, 9, 9>::Identity();
    P_ = (I - K * H_) * P_;
}

Eigen::Vector3d KFfilter::getPosition() const { return X_.segment<3>(0); }
Eigen::Vector3d KFfilter::getVelocity() const { return X_.segment<3>(3); }
Eigen::Vector3d KFfilter::getAcceleration() const { return X_.segment<3>(6); }

} // namespace mpcc