//类特点：加速度指令 -> 姿态+推力设定 转换器（离线仿真 mpc_offline_sim 的"飞控内环"入口），
//       增量非线性动态逆（INDI）实现，移植自 indi 工程的 INDI_control；有状态（滤波基准与推力积分）
//算法（每拍 update 一次）：
//  1. 横向增量：Δa_xy = a_des_xy − a_meas_xy（去重力后的运动加速度误差），
//     经加速度-姿态雅可比逆映射为 Δ(φ,θ)：Δ(φ,θ) = M⁻¹·Δa_xy / f，
//     M = ∂(a_x,a_y)/∂(φ,θ) / f 由测量姿态解析求得（同 indi 的 get_INDI_Matrix），f 为测量比力模值（当前推力）
//  2. 姿态设定 = 滤波后测量姿态（9Hz 一阶低通，对齐加速度计的时延）+ Δ(φ,θ)；
//     航向直接取 yaw_sp（indi 原工程未设定航向，此处沿用本项目 PID 的偏航设定）
//  3. 推力增量：T = T_base + Kp·Δa_z/(cosφ·cosθ) + Ki·∫Δa_z，T_base 为 6Hz 低通后的上一拍推力指令
//约定：
//  a_des 为"不含重力"的机动加速度 [m/s²]（ENU）；a_meas 为加速度计比力 [m/s²]（ENU，悬停≈+g）；
//  推力输出为机体z比力 [m/s²]（悬停≈g），直接喂 Mixer::allocate，与 drone_dynamic 质量归一化量纲一致；
//  推力增益由 indi 原值 0.0085（油门归一化量纲）换算得到（见 INDIParam），不宜取精确逆 1.0（积分饱和）；
//  倾角限幅防指令超出电机权限（超限时滚转/俯仰截断）；须按固定周期 dt 连续调用，不可跳拍
#ifndef MPCC_INDI_ATTITUDE_CONTROLLER_H
#define MPCC_INDI_ATTITUDE_CONTROLLER_H

#include <Eigen/Dense>
#include "Model/drone_dynamics.h"   // drone_dynamic: EulerAngle 等类型

namespace mpcc{

//转换输出：姿态设定 + 机体z比力设定
struct AttThrustSp {
    drone_dynamic::EulerAngle attSp; //姿态设定 ZYX 欧拉（FLU），直接喂 AttitudeController
    double thrustSp;                 //机体z比力设定 [m/s²]（悬停≈g），直接喂 Mixer::allocate
};

//INDI 参数（默认值移植自 indi 工程，量纲统一为 m/s²）
struct INDIParam {
    //推力比例增益：indi 原值 0.0085 为油门归一化量纲，按其 thrust_old 初值 0.4 作悬停油门换算为 m/s²：0.0085·g/0.4
    //注意 P 项经推力低通反馈后等效为积分增益 kp·ω_thrust，取 1.0 时（≈38/s）在高前飞推力损失下会积分饱和
    double kp_thrust      = 0.21;
    double ki_thrust      = 0.0;  //推力积分增益（同 indi 原工程，默认 0）
    double thrust_int_lim = 1.0;  //推力积分项限幅 [m/s²]
    double att_lpf_hz     = 9.0;  //测量姿态滤波截止频率 [Hz]（indi 原值 9，作为增量基准）
    double thrust_lpf_hz  = 6.0;  //推力指令滤波截止频率 [Hz]（indi 原值 6，作为增量基准）
    double max_tilt_deg   = 60.0; //倾角限幅 [deg]，超限截断滚转/俯仰设定
};

class INDIAttitudeController {
public:
    //gravity_val：模型重力加速度（取 drone_dynamic 参数值，保持与动力学一致）
    //注意：形参不用 gravity 命名——mpcc config.h 中 gravity 是宏，会打坏声明
    INDIAttitudeController(double gravity_val, const INDIParam &param = INDIParam{});

    //主更新（有状态，须按固定周期 dt 每拍调用）
    //  a_des_enu：机动加速度指令（ENU，不含重力）[m/s²]
    //  a_meas_enu：加速度计比力（ENU，悬停≈+g）[m/s²]
    //  att_meas：当前测量姿态（ZYX 欧拉，FLU）
    //  yaw_sp：偏航设定 [rad]
    AttThrustSp update(const Eigen::Vector3d &a_des_enu, const Eigen::Vector3d &a_meas_enu,
                       const drone_dynamic::EulerAngle &att_meas, double yaw_sp, double dt);

private:
    double gravity_;
    INDIParam param_;
    bool initialized_;     //滤波基准是否已用测量姿态初始化（首拍生效）
    double roll_base_;     //滤波后测量滚转（INDI 增量基准）[rad]
    double pitch_base_;    //滤波后测量俯仰（INDI 增量基准）[rad]
    double thrust_base_;   //滤波后推力指令（INDI 增量基准）[m/s²]，初值取悬停比力 g
    double thrust_int_;    //推力积分项 [m/s²·s]
};

}
#endif //MPCC_INDI_ATTITUDE_CONTROLLER_H
