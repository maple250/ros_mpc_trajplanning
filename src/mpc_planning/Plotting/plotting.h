#ifndef MPCC_PLOTTING_H
#define MPCC_PLOTTING_H

#include "config.h"
#include "types.h"
#include <matplotlibcpp.h>
#include <vector>
#include <array>
#include <string>
#include <map>
#include "MPC/mpc.h"

namespace plt = matplotlibcpp;

namespace mpcc {
class Plotting {
public:
    Plotting(double Ts, const PathToJson& path);

    // 战术拦截专用绘图函数：同时绘制我方实际轨迹、敌方实际轨迹、以及特定时刻的MPC预测段
    // motor_log: 四电机 PWM(µs) 时间序列（与 ego_log 同频）
    // motor_pwm_is_real: true = 来自 /mavros/rc/out 真实反馈；false = 由 MPC 加速度指令估算
    // mpc_accel_log / pid_accel_log: MPC 首点加速度指令与底层 PID 平滑指令（ENU，与 ego_log 同频）
    void plotIntercept(const std::vector<State>& ego_log,
                       const std::vector<TargetState>& target_log,
                       const std::vector<TrackPackage>& plan_log,
                       const std::vector<std::array<double,4>>& motor_log,
                       bool motor_pwm_is_real,
                       const std::vector<std::array<double,3>>& mpc_accel_log,
                       const std::vector<std::array<double,3>>& pid_accel_log) const;

private:
    double Ts_; // 数据采样周期，用于时间轴
};

// X 型四旋翼（PX4 iris 输出序号：M1前右 M2后左 M3前左 M4后右）简化推力分配模型：
// 由 MPC 加速度指令估算各电机 PWM(1000~2000µs)。仅在飞控无电机反馈时作为降级显示。
std::array<double,4> estimateMotorPWM(const Input& a_cmd);
}
#endif //MPCC_PLOTTING_H
