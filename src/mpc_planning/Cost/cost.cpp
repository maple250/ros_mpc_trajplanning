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
// 注意入参为预测目标状态 target_pred 和 目标航向向量 n_t
// QP 代价形式：J = 0.5*x^T Q x + f^T x + 0.5*u^T R u + r^T u
CostMatrix Cost::getCost(const TargetState &target_pred, const Eigen::Vector3d &n_t,
                         const State &x, const Input &u, int k) const
{
    (void)u; (void)k; // 当前代价不直接依赖输入本身与步数，保留接口
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

    // 敌我相对几何：LOS 视线（指向目标）
    Eigen::Vector3d r_los = target_pred.p_t - p_ego;
    const double rho = r_los.norm();
    // rho 趋于零时 LOS 方向退化，取 x 轴单位向量仅作数值保护
    Eigen::Vector3d n_los = (rho > 1e-3) ? (r_los / rho) : Eigen::Vector3d(1.0, 0.0, 0.0);

    // 投影算子（n_t 为目标航向单位向量，允许为零向量）
    Eigen::Matrix3d P_along = n_t * n_t.transpose();//沿 n_t 方向的投影算子
    Eigen::Matrix3d P_cross = Eigen::Matrix3d::Identity() - P_along;//垂直于 n_t 的投影矩阵

    //==================== 1. 法向位置误差收敛项 ====================
    // e_c = (I - n_t n_t^T)(p - p_t),  J_c = e_c^T Qc e_c,  Qc = q_c * I
    // 展开后 Q_p = q_c*P_cross, f_p = -Q_p * p_t
    // P_cross = Eigen::Matrix3d::Identity();
    const double w_v_track = 0.5 * (1.0 - std::tanh((rho - cost_param_.rho_v) / cost_param_.k_v));
    Eigen::Matrix3d Q_p = cost_param_.q_c * w_v_track * P_cross;
    cost.Q.block<3,3>(0,0) = Q_p;
    cost.f.head(3) = -Q_p * w_v_track * target_pred.p_t;

    //==================== 2. 速度大小控制项（高斯变权重） ====================
    // theta = angle(LOS, v_t)：迎面(theta->pi)权重最大，追击(theta->0)权重最小
    // w_v(theta) = q_vmag * exp(-(theta-pi)^2 / (2*sigma_v^2))，对称轴 theta=pi 处取峰值
    // 代价 J_v = -w_v * |v_ego|，一阶展开 |v| ≈ |v_lin| + v̂^T (v - v_lin)，
    // 常数项丢弃后线性项进入 f：f_v += -w_v * v̂
    double theta_los_vt = M_PI; // 夹角不可解时按迎面态势处理（权重最大）
    const double v_t_norm = target_pred.v_t.norm();
    if (rho > 1e-3 && v_t_norm > 1e-3) {
        double cos_theta = std::max(-1.0, std::min(1.0, n_los.dot(target_pred.v_t) / v_t_norm));
        theta_los_vt = std::acos(cos_theta);
    }
    const double w_vmag = cost_param_.q_vmag *
        std::exp(-std::pow(theta_los_vt - M_PI, 2) / (2.0 * cost_param_.sigma_v * cost_param_.sigma_v));
    // |v| 的梯度方向：v 趋于零时退化为 LOS 方向（鼓励朝目标加速）
    Eigen::Vector3d n_speed = (v_ego.norm() > 1e-3) ? v_ego.normalized() : n_los;
    cost.f.segment<3>(3) += -w_vmag * n_speed;

    //==================== 3. 基于 APN 制导律的速度矢量增量项 ====================
    // LOS 视线角速度（矢量形式）：lambda_dot_vec = (r x v_rel) / rho^2
    // APN 指令：a_apn = N * |v_rel| * |lambda_dot| * n_perp，n_perp = (r x v_rel) 方向单位向量
    // 代价 J_dv = -w_dv(rho) * dt * a_apn^T u，即奖励速度增量 dt*u 与 APN 指令方向对齐
    // 变权重：距离越小权重越大（近距末段强化 APN 修正），距离越大权重趋零
    // w_dv(rho) = q_dv * 0.5*(1 - tanh((rho - rho_dv)/k_dv))
    Eigen::Vector3d v_rel = v_ego - target_pred.v_t;
    Eigen::Vector3d w_los_vec = r_los.cross(v_rel); // = lambda_dot_vec * rho^2
    const double lam_dot = w_los_vec.norm() / (rho * rho + 1e-9);
    const double w_dv = cost_param_.q_dv *
        0.5 * (1.0 + std::tanh((rho - cost_param_.rho_dv) / cost_param_.k_dv));
    if (w_los_vec.norm() > 1e-6) {
        Eigen::Vector3d n_perp = w_los_vec.normalized();
        Eigen::Vector3d a_apn = cost_param_.N_apn * v_rel.norm() * lam_dot * n_perp;
        cost.r += -w_dv * Ts * a_apn;
    }

    //==================== 4. 速度跟踪二次惩罚项（tanh 变权重） ====================
    // 相对距离越大权重越大：w_v_track(rho) = q_v * 0.5*(1 + tanh((rho - rho_v)/k_v))
    // const double w_v_track = 0.5 * (1.0 - std::tanh((rho - cost_param_.rho_v) / cost_param_.k_v));
    // 期望速度方向：沿用原投影几何（追击/迎头自适应）
    Eigen::Vector3d p_proj = Eigen::Vector3d::Zero();
    if(n_t == Eigen::Vector3d::Zero()){
        p_proj = p_ego;
    }
    else{
        p_proj = target_pred.p_t + (n_t.dot(p_ego - target_pred.p_t)) * n_t;
    }
    Eigen::Vector3d vec_proj2target = target_pred.p_t - p_proj;
    Eigen::Vector3d v_ref = Eigen::Vector3d::Zero();
    double V_approach = 25.0;
    if(vec_proj2target.norm() > 1e-3)
    {
        v_ref = V_approach * vec_proj2target.normalized();
    }
    else if(vec_proj2target.dot(n_t) >= 0) // 投影-目标矢量与目标速度共向，视为追击问题
    {
        v_ref = V_approach * n_t;
    }
    else // 其他情况,非追击，对撞
    {
        v_ref = -V_approach * n_t;
    }
    Eigen::Matrix3d Q_v = cost_param_.q_v * w_v_track * Eigen::Matrix3d::Identity();
    cost.Q.block<3,3>(3,3) = Q_v;
    cost.f.segment<3>(3) += -Q_v * v_ref;

    // 正则化，保证 Q 正定
    Q_MPC Q_reg = 1e-12*Q_MPC::Identity();
    cost.Q = cost.Q + Q_reg;

    //==================== 5. 加速度指令惩罚项 ====================
    cost.R = cost_param_.q_a * Eigen::Matrix3d::Identity();

    return cost;
}



}
