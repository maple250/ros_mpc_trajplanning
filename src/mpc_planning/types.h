#ifndef MPCC_TYPES_H
#define MPCC_TYPES_H

#include "config.h"
namespace mpcc{
struct State{
    double px;
    double py;
    double pz;
    double vx;
    double vy;
    double vz;

    void setZero()
    {
        px = 0.0;
        py = 0.0;
        pz = 0.0;
        vx = 0.0;
        vy = 0.0;
        vz = 0.0;
    }
};

struct Input{
    double ax;
    double ay;
    double az;

    void setZero()
    {
        ax = 0.0;
        ay = 0.0;
        az = 0.0;
    }
};

// 目标当前状态 (由感知/通信模块在 t=0 时刻提供)
struct TargetState {
    Eigen::Vector3d p_t; // 目标当前位置
    Eigen::Vector3d v_t; // 目标当前速度
    Eigen::Vector3d a_t = Eigen::Vector3d::Zero(); // 目标当前加速度 (若没有可暂时设为零)
};

// 标准输出轨迹包
typedef struct TrackPackage {
    int track_number;
    std::vector<double> track_planning_time;
    std::vector<Eigen::Vector3d> track_planning_p;
    std::vector<Eigen::Vector3d> track_planning_v;
    std::vector<Eigen::Vector3d> track_planning_a;
} TrackPackage;
typedef struct FCUControlProtocol {
    int horizontal_mode = 1;
    int vertical_mode = 1;
    int heading_mode = 1;
    double position_command[3];
    double velocity_command[3];
    double attitude_command[3];
    double acceleration_command[3];
} FCUControlProtocol;

struct PathToJson{
    const std::string cost_path;
    const std::string bounds_path;
    const std::string normalization_path;
    const std::string config_path;
};
//Eigen 中的矩阵类型模板，定义矩阵/向量类型
typedef Eigen::Matrix<double,NX,1> StateVector;
typedef Eigen::Matrix<double,NU,1> InputVector;
typedef Eigen::Matrix<double,3,3> RotationMatrix;
//for final Lin-discretize-Model
typedef Eigen::Matrix<double,NX,NX> A_MPC;
typedef Eigen::Matrix<double,NX,NU> B_MPC;
typedef Eigen::Matrix<double,NX,1> g_MPC;

typedef Eigen::Matrix<double,NX,NX> Q_MPC;//二次规划状态二次型矩阵
typedef Eigen::Matrix<double,NU,NU> R_MPC;//二次规划输入二次型矩阵
typedef Eigen::Matrix<double,NX,NU> S_MPC;//soft constraint

typedef Eigen::Matrix<double,NX,1> f_MPC;//二次规划状态线性项矩阵
typedef Eigen::Matrix<double,NU,1> r_MPC;//二次规划输入线性项矩阵

typedef Eigen::Matrix<double,NPC,NX> C_MPC;//  “lb ≤   C*x + D*u   ≤ ub” 中的C，可以描述和x相关的NPC个约束
typedef Eigen::Matrix<double,1,NX> C_i_MPC;//  “lb ≤ Ci*xi + Di*ui ≤ ub” 中的Ci，可以描述和x相关的1个约束
typedef Eigen::Matrix<double,NPC,NU> D_MPC;//  “lb ≤   C*x + D*u   ≤ ub” 中的D，可以描述和u相关的NPC个约束
typedef Eigen::Matrix<double,NPC,1> d_MPC;//   NPC个（全部线性不等式约束数量）约束两端边界值：lb、ub

typedef Eigen::Matrix<double,NS,NS> Z_MPC;
typedef Eigen::Matrix<double,NS,1> z_MPC;

typedef Eigen::Matrix<double,NX,NX> TX_MPC;//状态归一化对角阵
typedef Eigen::Matrix<double,NU,NU> TU_MPC;//输入归一化对角阵
typedef Eigen::Matrix<double,NS,NS> TS_MPC;//软约束归一化对角阵

typedef Eigen::Matrix<double,NX,1> Bounds_x;//状态边界
typedef Eigen::Matrix<double,NU,1> Bounds_u;//输入边界
typedef Eigen::Matrix<double,NS,1> Bounds_s;//软约束边界

struct LinModelMatrix {
    A_MPC A;
    B_MPC B;
    g_MPC g;
};
struct ConstrainsMatrix {
    // dl <= C xk + D uk <= du
    C_MPC C;    //polytopic state constraints
    D_MPC D;    //polytopic input constraints
    d_MPC dl;   //lower bounds
    d_MPC du;   //upper bounds
};
StateVector stateToVector(const State &x);
InputVector inputToVector(const Input &u);

State vectorToState(const StateVector &xk);
Input vectorToInput(const InputVector &uk);

State arrayToState(double *xk);
Input arrayToInput(double *uk);
}
#endif //MPCC_TYPES_H
