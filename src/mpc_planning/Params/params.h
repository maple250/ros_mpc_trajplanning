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
    double q_l;
    double q_c;
    double q_v;
    double q_a;
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
    InitialParam();
    InitialParam(std::string file);
};
}
#endif //MPCC_PARAMS_H
