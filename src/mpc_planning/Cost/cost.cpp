#include "cost.h"
#include <cmath>
#include <algorithm>
namespace mpcc{
Cost::Cost() 
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Cost::Cost(const PathToJson &path) 
:cost_param_(CostParam(path.cost_path))
{}
// 注意入参变为了预测目标状态 target_pred 和 航向向量 n_t
CostMatrix Cost::getCost(const TargetState &target_pred, const Eigen::Vector3d &n_t, 
                         const State &x, const Input &u, int k) const 
{
    CostMatrix cost;
    cost.Q = Q_MPC::Zero();
    cost.R = R_MPC::Zero();
    cost.S = S_MPC::Zero();
    cost.f = f_MPC::Zero();
    cost.r = r_MPC::Zero();
    cost.Z = Z_MPC::Zero();
    cost.z = z_MPC::Zero();
    
    Eigen::Vector3d p_ego(x.px, x.py, x.pz);
    Eigen::Vector3d v_ego(x.vx, x.vy, x.vz);

    // 1. 滑动虚拟导引点计算
    // double Target_Vel_norm = n_t.norm();
    Eigen::Vector3d p_proj = Eigen::Vector3d::Zero();
    if(n_t == Eigen::Vector3d::Zero()){
        p_proj = p_ego;
    }
    else{
        p_proj = target_pred.p_t + (n_t.dot(p_ego - target_pred.p_t)) * n_t;
    }
    double L = 2.0 * v_ego.norm() + 80.0; // 动态前瞻距离
    //投影点到目标的方向向量
    Eigen::Vector3d vec_proj2target = target_pred.p_t - p_proj;
    Eigen::Vector3d v_ref = Eigen::Vector3d::Zero(); // 迎头对撞方向
    Eigen::Vector3d p_ref = Eigen::Vector3d::Zero(); // 参考位置
    double V_approach = 25.0; 

    if(vec_proj2target.norm() > 1e-3) // 避免除以零
    {
        v_ref = V_approach * vec_proj2target.normalized();//追击
        p_ref = p_proj + L * vec_proj2target.normalized(); 
    }
    else if(vec_proj2target.dot(n_t) >= 0) // 投影-目标矢量与目标速度共向，视为追击问题
    {
        v_ref = V_approach * n_t;
        p_ref = p_proj + L * n_t; 
    }
    else // 其他情况,非追击，仍然对撞
    {
        v_ref = -V_approach * n_t;
        p_ref = p_proj - L * n_t; 
    }
    // 投影矩阵与极端权重
    Eigen::Matrix3d P_along = n_t * n_t.transpose();//沿n_t方向的投影算子
    Eigen::Matrix3d P_cross = Eigen::Matrix3d::Identity() - P_along;//垂直于n_t的投影矩阵
    
    // 权重：法向极高(1000)，切向适中(10)
    Eigen::Matrix3d Q_p = cost_param_.q_l * P_along + cost_param_.q_c * P_cross; 
    cost.Q.block<3,3>(0,0) = Q_p;

    // 速度对齐惩罚矩阵50.0
    Eigen::Matrix3d Q_v = cost_param_.q_v * Eigen::Matrix3d::Identity();
    cost.Q.block<3,3>(3,3) = Q_v;
    Q_MPC Q_reg = 1e-12*Q_MPC::Identity();
    cost.Q = cost.Q + Q_reg;

    // 4. 一次项 f 计算 (J = 0.5*x^T Q x + f^T x)
    // 展开 0.5*(x-x_ref)^T Q (x-x_ref) 得到 f = -Q * x_ref
    cost.f.head(3) = -Q_p * p_ref;
    cost.f.tail(3) = -Q_v * v_ref;

    // 5. 控制能量惩罚2.0
    cost.R = cost_param_.q_a * Eigen::Matrix3d::Identity(); // 适度惩罚加速度急变

    return cost;
}



}