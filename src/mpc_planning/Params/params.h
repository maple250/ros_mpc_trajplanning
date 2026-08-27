//类特点：成员变量粗分类、存储参数，成员函数负责从json文件读取参数
#ifndef MPCC_PARAMS_H
#define MPCC_PARAMS_H

// #include <iostream>
// #include <fstream>
#include <vector>
#include <nlohmann/json.hpp>
#include "config.h"
#include "types.h"

namespace mpcc{
//used namespace
using json = nlohmann::json;
//代价函数参数
class CostParam{
public:
    // 1. 法向位置误差收敛项：e_c = (I-n n^T)(p-p_t)，J = e_c^T Qc e_c
    double q_c;
    // 4. 速度跟踪二次项固定权重（随距离调制：距离越大权重越大）
    double q_v;
    // 5. 加速度指令惩罚权重
    double q_a;
    // 2. 速度大小奖励项峰值权重（LOS 与 v_t 夹角 θ=π 迎面处最大，高斯衰减）
    double q_vmag;
    // 2. 速度大小奖励权重的高斯宽度 (rad)
    double sigma_v;
    // 3. APN 速度增量项固定权重（随距离调制：距离越小权重越大）
    double q_dv;
    // 3. APN 制导系数 N
    double N_apn;
    // 3. APN 权重 tanh 过渡中心距离与过渡宽度
    double rho_dv;
    double k_dv;
    // 4. 速度跟踪权重 tanh 过渡中心距离与过渡宽度
    double rho_v;
    double k_v;
    CostParam();
    CostParam(std::string file);
};
//状态量和控制量的上下界参数
class BoundsParam{
public:
    struct LowerStateBounds{
        double px_l;
        double py_l;
        double pz_l;
        double vx_l;
        double vy_l;
        double vz_l;
    };
    struct UpperStateBounds{
        double px_u;
        double py_u;
        double pz_u;
        double vx_u;
        double vy_u;
        double vz_u;
    };
    //命名规定：变量不包括后缀时（如上下界），本身最多只能有一个“_”  !!!
    struct LowerInputBounds{
        double ax_l;
        double ay_l;
        double az_l;
    };
    struct UpperInputBounds{
        double ax_u;
        double ay_u;
        double az_u;
    };
    LowerStateBounds lower_state_bounds;
    UpperStateBounds upper_state_bounds;
    LowerInputBounds lower_input_bounds;
    UpperInputBounds upper_input_bounds;
    BoundsParam();
    BoundsParam(std::string file);
};
//状态量和控制量的归一化参数（与边界相关）
class NormalizationParam{
public:
    TX_MPC T_x;
    TX_MPC T_x_inv;

    TU_MPC T_u;
    TU_MPC T_u_inv;

    TS_MPC T_s;
    TS_MPC T_s_inv;

    NormalizationParam();
    NormalizationParam(std::string file);
};
class InitialParam{
public:
    Eigen::Vector3d Pos_target_init;
    Eigen::Vector3d Vel_target_init;
    Eigen::Vector3d Pos_self_init;
    Eigen::Vector3d Vel_self_init;
    //目标运动特性参数（由速度大小+方位角描述，Vel_target_init 换算得到）
    double Target_vel_h;     //水平（惯性系）合速度大小 (m/s)
    double Target_vel_v;     //垂直（惯性系）速度 (m/s)，负值代表下降
    double Target_theta;     //水平速度方位角 (deg)，逆时针为正
    int Target_movetype;     //运动模式：1 匀速直线，2 匀速圆周，3 预留拓展
    double Target_circle_R;  //匀速圆周运动半径 (m)，正=逆时针盘旋，负=顺时针盘旋
    InitialParam();
    InitialParam(std::string file);
};
}
#endif //MPCC_PARAMS_H
