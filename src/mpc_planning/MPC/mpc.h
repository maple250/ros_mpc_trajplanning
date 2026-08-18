#ifndef MPCC_MPC_H
#define MPCC_MPC_H

#include "config.h"
#include "types.h"
#include "Params/params.h"
// #include "Spline/arc_length_spline.h"
// #include "Model/model.h"
// #include "Model/integrator.h"
#include "Cost/cost.h"
// #include "Constraints/constraints.h"
#include "Constraints/bounds.h"
#include "Plotting/plotting.h"

#include "Interfaces/solver_interface.h"
#include "Interfaces/hpipm_interface.h"

#include <array>
#include <memory>
#include <ctime>
#include <ratio>
#include <chrono>

namespace mpcc{
//定义 MPC 优化变量结构体，包含状态和输入变量
struct OptVariables {
    State xk;
    Input uk;
};
//优化输入:定义 MPC 每个时间步的结构体，包含线性离散化模型、代价矩阵、不等式约束矩阵以及变量边界信息
struct Stage {
    LinModelMatrix lin_model;//{19*19 5*5 19*1}:线性离散化模型，构建等式约束
    CostMatrix cost_mat;//{19*19 5*5 19*5 19*1 5*1 ns*ns ns*1}
    ConstrainsMatrix constrains_mat;//{C:ng*19 D:ng*5 dl:ng*1 du:ng*1}不等式约束矩阵，构建不等式约束

    Bounds_x u_bounds_x;//19*1
    Bounds_x l_bounds_x;//19*1

    Bounds_u u_bounds_u;//5*1
    Bounds_u l_bounds_u;//5*1

    Bounds_s u_bounds_s;//0*1:软约束边界
    Bounds_s l_bounds_s;//0*1

    //nx    -> number of states
    //nu    -> number of inputs
    //nbx   -> number of bounds on x
    //nbu   -> number of bounds on u
    //ng    -> number of polytopic constratins
    //ns   -> number of soft constraints
    int nx, nu, nbx, nbu, ng, ns;
};
//优化输出:定义 MPC 返回结构体，包含控制输入、MPC 预测时域内的状态和输入变量以及总计算时间
struct MPCReturn {
    const Input u0;//第一步的最优输入，用于给到系统进行控制
    const std::array<OptVariables,N+1> mpc_horizon;//一个包含 N+1 个元素的容器，每个元素是一个 OptVariables 对象，用于记录 MPC 预测时域内的状态和输入变量
    const double time_total;
};
//
class MPC {
public:
    MPC();
    MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts, const PathToJson &path);
    int solver_status = -1;
    bool traj_finish = false;// --- 跟踪状态标志位 ---
    InitialParam state_param_;
    TrackPackage runInterceptMPC(const State &x0, const TargetState &target_current, double offboard_time);
    void logData(const State &x, const TargetState &target, const TrackPackage &plan);
    // void logPlot();
    void logPlot(const double Ts_, const PathToJson &json_paths);
    void reached_detection(const State &x, const TargetState &x_t, double offboard_time_, const PathToJson &json_paths);
private:
    bool valid_initial_guess_;
    std::array<Stage, N + 1> stages_;////优化输入：预测时域内每个时间步的结构体数组
    std::array<OptVariables, N + 1> initial_guess_;
    std::array<OptVariables, N + 1> optimal_solution_;

    void setStage(const State &xk, const Input &uk, const TargetState &target_pred, const Eigen::Vector3d &n_t, const int time_step);

    CostMatrix normalizeCost(const CostMatrix &cost_mat);
    LinModelMatrix normalizeDynamics(const LinModelMatrix &lin_model);
    // 连续模型 p'=v, v'=u-drag_coeff*v 的精确 ZOH 离散化（setStage 与 updateInitialGuess 共用，保证一致）
    LinModelMatrix discretizeModel() const;
    std::array<OptVariables,N+1> deNormalizeSolution(const std::array<OptVariables,N+1> &solution);
    void updateInitialGuess(const State &x0);
    void generateNewInitialGuess(const State &x0);
    std::array<OptVariables, N + 1> sqpSolutionUpdate(const std::array<OptVariables, N + 1> &last_solution,
                                                      const std::array<OptVariables, N + 1> &current_solution);
    int n_sqp_;
    double sqp_mixing_;
    int n_non_solves_;
    int n_no_solves_sqp_;
    int n_reset_;
    const double Ts_;

    // Plotting plotter;
    Cost cost_;
    Bounds bounds_;//成员变量是Bounds类对象，需要在构造函数中传入BoundsParam类对象进行初始化（参考Bounds构造函数）
    NormalizationParam normalization_param_;

    std::unique_ptr<SolverInterface> solver_interface_;

    //日志记录变量
    std::vector<State> ego_log;
    std::vector<TargetState> target_log;
    std::vector<TrackPackage> plan_log;
    int step_counter = 0; // 用于降采样记录规划轨迹
};

}

#endif //MPCC_MPC_H
