#ifndef KFFILTER_H
#define KFFILTER_H

#include <Eigen/Dense>
#include <iostream>

namespace mpcc {

class KFfilter {
public:
    KFfilter();
    
    // 初始化滤波器状态
    void init(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0);
    
    // 10Hz 高频预测步：由 MPC 线程调用，推演状态
    void predict(double dt);
    
    // 0.5Hz 低频更新步：由传感器线程调用，融合新观测值
    void update(const Eigen::Vector3d& p_meas, const Eigen::Vector3d& v_meas);
    
    // 获取当前滤波后的状态
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    Eigen::Vector3d getAcceleration() const;

private:
    Eigen::Matrix<double, 9, 1> X_; // 状态向量 [px, py, pz, vx, vy, vz, ax, ay, az]^T
    Eigen::Matrix<double, 9, 9> P_; // 状态协方差矩阵
    Eigen::Matrix<double, 6, 9> H_; // 观测矩阵 (只能观测到 p 和 v)
    Eigen::Matrix<double, 6, 6> R_; // 观测噪声协方差
    Eigen::Matrix<double, 9, 9> Q_; // 过程噪声协方差
};

} // namespace mpcc
#endif // KFFILTER_H