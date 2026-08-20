#include "mpc.h"

namespace mpcc{
MPC::MPC()
:Ts_(1.0)
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

MPC::MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts, const PathToJson &path)
: state_param_(InitialParam(path.config_path)),
  valid_initial_guess_(false),          // 1. 布尔值
  Ts_(Ts),                              // 2. 常量时间步
//   plotter(Ts, path);                    // 3. plot
  cost_(Cost(path)),                    // 4. 代价函数
  bounds_(BoundsParam(path.bounds_path)),// 5. 边界
  normalization_param_(NormalizationParam(path.normalization_path)), // 6. 归一化参数(即便没用也按顺序放)
  
  solver_interface_(new HpipmInterface()) // 7. 求解器指针通常放最后
{
    n_sqp_ = n_sqp;
    sqp_mixing_ = sqp_mixing;
    n_non_solves_ = 0;
    n_no_solves_sqp_ = 0;
    n_reset_ = n_reset;
}

// MPC 运行主入口
TrackPackage MPC::runInterceptMPC(const State &x0, const TargetState &target_current, double offboard_time) {
    // auto t1 = std::chrono::high_resolution_clock::now();
    n_no_solves_sqp_ = 0;//求解器发散次数
    //输入N+1步的 initial_guess_ ，输出完整的Stage结构体stages_（N+1步的线性化初始条件）
    Eigen::Vector3d p_pred = target_current.p_t;
    Eigen::Vector3d v_pred = target_current.v_t;
    Eigen::Vector3d a_pred = target_current.a_t;// 这里的 a_current 是通过 KF
    
    const double tau = 1.0; 
    const double decay_factor = std::exp(-Ts_ / tau);//0.9
    if(valid_initial_guess_){
            updateInitialGuess(x0);
        }
    else{
            generateNewInitialGuess(x0);
        }
    for(int i=0;i<n_sqp_;i++)//正式计算前，先多次（2轮）迭代、混合状态，计算一个初始序列用于线性化
    {
        for(int k=0;k<=N;k++)//0到N，共N+1
        {
            TargetState target_pred_k;
            target_pred_k.p_t = p_pred;
            target_pred_k.v_t = v_pred;
            target_pred_k.a_t = a_pred;
            // 计算当前预测步的瞬时航向vector
            double v_pred_norm = v_pred.norm();
            Eigen::Vector3d n_t_pred = (v_pred_norm > 1e-3) ? (v_pred / v_pred_norm) : Eigen::Vector3d(0,0,0);
            // 传入 Stage
            setStage(initial_guess_[k].xk, initial_guess_[k].uk, target_pred_k, n_t_pred, k);
        //===================Singer 模型机动===================//
            p_pred = p_pred + v_pred * Ts_ + 0.5 * a_pred * (Ts_ * Ts_);
            v_pred = v_pred + a_pred * Ts_;
            // 随着 k 的增加，a_pred 会逐渐趋于 0。预测轨迹将平滑过渡为匀速直线运动。
            a_pred = a_pred * decay_factor;
        }
        State x0_normalized = vectorToState(normalization_param_.T_x_inv*stateToVector(x0));
        optimal_solution_ = solver_interface_->solveMPC(stages_,x0_normalized, &this->solver_status);
        // std::cout << "solver done!" << std::endl;
        optimal_solution_ = deNormalizeSolution(optimal_solution_);
        if(this->solver_status != 0){
            std::cout << "Solve Warning!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            n_no_solves_sqp_++;
        } 
        initial_guess_ = sqpSolutionUpdate(initial_guess_, optimal_solution_);
    }
    const int max_error = std::max(n_sqp_-1,1);
    if(n_no_solves_sqp_ >= max_error)
        n_non_solves_++;//本次预热(含有n_sqp_步MPC求解)失败计数
    else
        n_non_solves_ = 0;
    //预热失败次数超过阈值n_reset_，重新生成初始猜测序列
    if(n_non_solves_ >= n_reset_){
        valid_initial_guess_ = false;
    }
    // 组装输出包
    TrackPackage result_packg;
    result_packg.track_number = N + 1;
    // 预分配内存，提升 vector 性能
    result_packg.track_planning_time.reserve(N + 1);
    result_packg.track_planning_p.reserve(N + 1);
    result_packg.track_planning_v.reserve(N + 1);
    result_packg.track_planning_a.reserve(N + 1);

    for (int k = 0; k <= N; ++k) {
        double t_k = offboard_time + k * Ts_;
        result_packg.track_planning_time.push_back(t_k);
        if (std::isnan(optimal_solution_[k].xk.px) || std::isnan(optimal_solution_[k].uk.ax)) {
            // 如果发生内存灾难，强行输出“原地悬停”的安全指令给到底层 PID
            result_packg.track_planning_p.push_back(Eigen::Vector3d(x0.px, x0.py, x0.pz));
            result_packg.track_planning_v.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            if (k < N) result_packg.track_planning_a.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            else result_packg.track_planning_a.push_back(result_packg.track_planning_a.back());
            
            // 仅在 k=0 打印一次警告
            if (k == 0) std::cout << "[MPC SHIELD] NaN detected! Emitting safety hover commands." << std::endl;
        } else {
            // 正常情况下的赋值
            Eigen::Vector3d p_k(optimal_solution_[k].xk.px, optimal_solution_[k].xk.py, optimal_solution_[k].xk.pz);
            Eigen::Vector3d v_k(optimal_solution_[k].xk.vx, optimal_solution_[k].xk.vy, optimal_solution_[k].xk.vz);
            Eigen::Vector3d a_k(optimal_solution_[k].uk.ax, optimal_solution_[k].uk.ay, optimal_solution_[k].uk.az);
            
            result_packg.track_planning_p.push_back(p_k);
            result_packg.track_planning_v.push_back(v_k);
            if (k < N) {
                result_packg.track_planning_a.push_back(a_k);
            } 
            else {
                result_packg.track_planning_a.push_back(result_packg.track_planning_a.back());
            }
        }
    }
    // 性能耗时统计
    // auto t2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "MPC Time: " << time_span.count() * 1000.0 << " ms" << std::endl;
    return result_packg;
}

void MPC::logData(const State &x, const TargetState &target, const TrackPackage &plan,
                  const std::array<double,4> &motor_pwm, const std::array<double,3> &pid_accel)
{
    // 降采样记录规划轨迹
    ego_log.push_back(x);
    target_log.push_back(target);
    if (step_counter % 25 == 0) {
        plan_log.push_back(plan);
    }
    // 当前时刻 MPC 加速度指令（预测轨迹首点）
    Input a_cmd;
    a_cmd.setZero();
    if (!plan.track_planning_a.empty()) {
        a_cmd.ax = plan.track_planning_a.front().x();
        a_cmd.ay = plan.track_planning_a.front().y();
        a_cmd.az = plan.track_planning_a.front().z();
    }
    mpc_accel_log.push_back({a_cmd.ax, a_cmd.ay, a_cmd.az});
    pid_accel_log.push_back(pid_accel);
    // 电机数据：优先使用飞控真实 PWM 反馈(/mavros/rc/out)；
    // 反馈为全零时，用当前时刻 MPC 加速度指令经简化分配模型估算
    const bool has_real = motor_pwm[0] != 0.0 || motor_pwm[1] != 0.0 ||
                          motor_pwm[2] != 0.0 || motor_pwm[3] != 0.0;
    if (has_real) {
        motor_log.push_back(motor_pwm);
        motor_pwm_is_real_ = true;
        last_real_pwm_ = motor_pwm;
    } else if (motor_pwm_is_real_) {
        motor_log.push_back(last_real_pwm_); // 反馈丢帧，保持上一帧
    } else {
        motor_log.push_back(estimateMotorPWM(a_cmd));
    }
    step_counter++;
}
void MPC::logPlot(const double Ts_, const PathToJson &json_paths)
{
    Plotting plotter(Ts_, json_paths);
    plotter.plotIntercept(ego_log, target_log, plan_log, motor_log, motor_pwm_is_real_,
                          mpc_accel_log, pid_accel_log);
}
void MPC::reached_detection(const State &x, const TargetState &x_t, double offboard_time_, const PathToJson &json_paths)
{
    double dist = (Eigen::Vector3d(x.px, x.py, x.pz) - x_t.p_t).norm();
    double Vel = Eigen::Vector3d(x.vx, x.vy, x.vz).norm();
    std::cout << "Time: " << offboard_time_ << "s | Dist: " << dist << "m | Vel: " << Vel << "m/s" << std::endl;
    if (dist <= 350.0) {
        std::cout << "[TARGET REACHED]" << std::endl;
        traj_finish = true;
        logPlot(Ts_,json_paths);
    }
}
//MPC-solver输入配置：输入“单步”的状态-输入，输出Stage结构体
void MPC::setStage(const State &xk, const Input &uk, const TargetState &target_pred, const Eigen::Vector3d &n_t, const int time_step)
{
    stages_[time_step].nx = NX;
    stages_[time_step].nu = NU;
    stages_[time_step].ng = 0;
    stages_[time_step].ns = 0;
    stages_[time_step].cost_mat = normalizeCost(cost_.getCost(target_pred, n_t, xk, uk, time_step));

    const LinModelMatrix lin_model = discretizeModel();
    stages_[time_step].lin_model = normalizeDynamics(lin_model);
    //将边界传入单步Stage结构体
    stages_[time_step].l_bounds_x = normalization_param_.T_x_inv.diagonal().cwiseProduct(bounds_.getBoundsLX());
    stages_[time_step].u_bounds_x = normalization_param_.T_x_inv.diagonal().cwiseProduct(bounds_.getBoundsUX());
    stages_[time_step].l_bounds_u = normalization_param_.T_u_inv.diagonal().cwiseProduct(bounds_.getBoundsLU());
    stages_[time_step].u_bounds_u = normalization_param_.T_u_inv.diagonal().cwiseProduct(bounds_.getBoundsUU());
    // stages_[time_step].l_bounds_s = normalization_param_.T_s_inv.diagonal().cwiseProduct(bounds_.getBoundsLS());
    // stages_[time_step].u_bounds_s = normalization_param_.T_s_inv.diagonal().cwiseProduct(bounds_.getBoundsUS());
}
//对代价矩阵进行归一化（乘上状态-输入边界对角矩阵，大值）
CostMatrix MPC::normalizeCost(const CostMatrix &cost_mat)
{
    const Q_MPC Q = normalization_param_.T_x.diagonal().asDiagonal() * cost_mat.Q * normalization_param_.T_x.diagonal().asDiagonal();
    const R_MPC R = normalization_param_.T_u.diagonal().asDiagonal() * cost_mat.R * normalization_param_.T_u.diagonal().asDiagonal();
    const f_MPC f = normalization_param_.T_x.diagonal().asDiagonal() * cost_mat.f;
    const r_MPC r = normalization_param_.T_u.diagonal().asDiagonal() * cost_mat.r;
    const Z_MPC Z = normalization_param_.T_s.diagonal().asDiagonal() * cost_mat.Z * normalization_param_.T_s.diagonal().asDiagonal();
    const z_MPC z = normalization_param_.T_s.diagonal().asDiagonal() * cost_mat.z;
    return {Q,R,S_MPC::Zero(),f,r,Z,z};
}
//对线性离散化模型进行归一化（原离散方程左乘T逆）
LinModelMatrix MPC::normalizeDynamics(const LinModelMatrix &lin_model)
{
    const A_MPC A = normalization_param_.T_x_inv.diagonal().asDiagonal() * lin_model.A * normalization_param_.T_x.diagonal().asDiagonal();
    const B_MPC B = normalization_param_.T_x_inv.diagonal().asDiagonal() * lin_model.B * normalization_param_.T_u.diagonal().asDiagonal();
    const g_MPC g = normalization_param_.T_x_inv.diagonal().asDiagonal() * lin_model.g;
    return {A,B,g};
}
//连续模型 p'=v, v'=u-drag_coeff*v 的精确 ZOH 离散化：
//  A = [[I, b_v*I],[0, adv*I]],  B = [[b_p*I],[b_v*I]]
//  其中 adv = e^{-c*Ts}, b_v = (1-adv)/c, b_p = (Ts-b_v)/c
LinModelMatrix MPC::discretizeModel() const
{
    LinModelMatrix lin_model;
    lin_model.A = Eigen::Matrix<double, NX, NX>::Identity();
    lin_model.B = Eigen::Matrix<double, NX, NU>::Zero();
    lin_model.g = Eigen::Matrix<double, NX, 1>::Zero();
    if (drag_coeff > 1e-9) {
        const double adv = std::exp(-drag_coeff * Ts_); // v 的自回归系数
        const double b_v = (1.0 - adv) / drag_coeff;    // v 对 u、p 对 v 的增益
        const double b_p = (Ts_ - b_v) / drag_coeff;    // p 对 u 的增益（Euler 离散化此项为 0）
        lin_model.A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * b_v;
        lin_model.A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * adv;
        lin_model.B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * b_p;
        lin_model.B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * b_v;
    } else {
        // 无阻尼退化：纯双积分器精确离散化
        lin_model.A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * Ts_;
        lin_model.B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.5 * Ts_ * Ts_);
        lin_model.B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * Ts_;
    }
    return lin_model;
}
//输入N+1步归一化后的MPC解（状态-输入），输出反归一化后的MPC解，并转换为 多个结构体容器array 形式
std::array<OptVariables,N+1> MPC::deNormalizeSolution(const std::array<OptVariables,N+1> &solution)
{
    std::array<OptVariables, N + 1> denormalized_solution;
    StateVector updated_x_vec;
    InputVector updated_u_vec;
    for (int i = 0; i <= N; i++) {
        updated_x_vec = normalization_param_.T_x.diagonal().cwiseProduct(stateToVector(solution[i].xk));//eigen向量形式
        updated_u_vec = normalization_param_.T_u.diagonal().cwiseProduct(inputToVector(solution[i].uk));
        denormalized_solution[i].xk = vectorToState(updated_x_vec);//最终转化为State结构体形式
        denormalized_solution[i].uk = vectorToInput(updated_u_vec);
    }
    return denormalized_solution;
}

//有疑问，对于输入序列的更新
void MPC::updateInitialGuess(const State &x0)
{
    for(int i=1;i<=N;i++)//从1到N，共N步 
        initial_guess_[i-1] = initial_guess_[i];//initial_guess_ 是N+1个xk和uk结构体的容器

    initial_guess_[0].xk = x0;
    initial_guess_[N-1].uk = initial_guess_[N-2].uk;// = initial_guess_[N-2].uk;

    // 与 setStage 使用同一离散化模型传播末步初始猜测（此前此处手写的 An 缺少 p+=v*Ts 项）
    const LinModelMatrix disc = discretizeModel();
    initial_guess_[N].xk = vectorToState( disc.A * stateToVector(initial_guess_[N-1].xk) + disc.B * inputToVector(initial_guess_[N-1].uk) );
    initial_guess_[N].uk.setZero();// = initial_guess_[N-2].uk;m脚本中这个位置为空，即输入序列永远比输出序列少一位
}
//输入初始状态，生成N+1步的初始猜测序列（如m脚本中的初始匀速直线前进假设）
void MPC::generateNewInitialGuess(const State &x0)
{
    for(int i = 0;i<=N;i++)
    {
        initial_guess_[i].xk = x0;
        initial_guess_[i].uk.setZero();
    }
    valid_initial_guess_ = true;
}

//基于上次的MPC解，进行SQP混合更新，应对非线性优化初期的数值不稳定问题，“预测跳变”可能使线性化点远离真实轨迹
std::array<OptVariables,N+1> MPC::sqpSolutionUpdate(const std::array<OptVariables,N+1> &last_solution,
                                                    const std::array<OptVariables,N+1> &current_solution)
{
    //TODO use line search and merit function
    std::array<OptVariables,N+1> updated_solution;
    StateVector updated_x_vec;
    InputVector updated_u_vec;
    for(int i = 0;i<=N;i++)
    {
        updated_x_vec = sqp_mixing_*stateToVector(current_solution[i].xk)
                        +(1.0-sqp_mixing_)*stateToVector(last_solution[i].xk);
        updated_u_vec = sqp_mixing_*inputToVector(current_solution[i].uk)
                        +(1.0-sqp_mixing_)*inputToVector(last_solution[i].uk);

        updated_solution[i].xk = vectorToState(updated_x_vec);
        updated_solution[i].uk = vectorToInput(updated_u_vec);
    }

    return updated_solution;
}
}