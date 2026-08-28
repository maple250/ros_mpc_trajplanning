//类特点：加速度指令 -> 姿态+推力设定 转换器（离线仿真 mpc_offline_sim 的"飞控内环"入口，
//       独立控制器文件，与 drone_dynamic 的 rate_controller/attitude_controller 同风格：无状态纯函数）
//算法：几何法（与 PX4 mc_pos_control 同源）——期望总比力 f_des = a_des + g·e3（ENU），
//      机体 z 轴对准 f_des 方向，航向由 x_c=[cos(yaw),sin(yaw),0] 张成，构造 R_des 后经
//      drone_dynamic::Frames 转 ZYX 欧拉角（FLU 语义，正 pitch=低头）；推力比力 = ||f_des||
//约定：输入 a_des 为"不含重力"的机动加速度（与在线版 MAVROS setpoint_raw 加速度语义一致），
//      单位 m/s²（与 drone_dynamic 质量归一化比力同量纲，零换算）；
//      倾角限幅防指令超权限（超限时水平分量等比缩放，垂向保持）
#ifndef MPCC_ACCEL_TO_ATTITUDE_CONTROLLER_H
#define MPCC_ACCEL_TO_ATTITUDE_CONTROLLER_H

#include <Eigen/Dense>
#include "Model/drone_dynamics.h"   // drone_dynamic: EulerAngle 等类型

namespace mpcc{

//转换输出：姿态设定 + 机体z比力设定
struct AttThrustSp {
    drone_dynamic::EulerAngle attSp; //姿态设定 ZYX 欧拉（FLU），直接喂 AttitudeController
    double thrustSp;                 //机体z比力设定 [m/s²]（悬停≈g），直接喂 Mixer::allocate
};

class AccelToAttitudeController {
public:
    //gravity_val：模型重力加速度（取 drone_dynamic 参数值，保持与动力学一致）
    //max_tilt_deg：倾角限幅 [deg]（默认 60°，超过后水平分量等比缩放）
    //注意：形参不用 gravity 命名——mpcc config.h 中 gravity 是宏，会打坏声明
    AccelToAttitudeController(double gravity_val, double max_tilt_deg = 60.0);

    //主更新：输入机动加速度指令（ENU，不含重力）与偏航设定 [rad]，输出姿态+推力设定
    //无状态 const：同一输入恒得同一输出，可任意频率调用
    AttThrustSp update(const Eigen::Vector3d &a_des_enu, double yaw_sp) const;

private:
    double gravity_;
    double max_tilt_;
};

}
#endif //MPCC_ACCEL_TO_ATTITUDE_CONTROLLER_H
