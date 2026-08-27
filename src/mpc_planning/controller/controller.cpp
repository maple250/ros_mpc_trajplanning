#include "controller/controller.h"
namespace mpcc {
// ================= Controller_Base 实现 =================
    void Controller_Base::set_Safe_Hovering(FCUControlProtocol& drone_ctrl)
    {
        // 直接在基类中赋值底层控制为：当前位置悬停
        drone_ctrl.horizontal_mode = 1;//水平1：位置控制
        drone_ctrl.vertical_mode = 1;//垂向1：位置控制
        drone_ctrl.heading_mode = 1;//偏航1：正常偏航角控制
        drone_ctrl.position_command[0] = state_feedbk.Pos_enu(0);//控制指令：ENU位置指令
        drone_ctrl.position_command[1] = state_feedbk.Pos_enu(1);
        drone_ctrl.position_command[2] = state_feedbk.Pos_enu(2);
        drone_ctrl.attitude_command[2] = state_feedbk.EulerAngles(2);//控制指令：偏航角位置指令
    }
    //检查轨迹包状态，return 0:正常,1:等待,2:超时
    int Controller_Base::CheckTrajectoryStatus(const TrackPackage& pkg, FCUControlProtocol& drone_ctrl)
    {
        // 1. 存储/更新轨迹包
        // 只有当包内有数据时才更新本地存储，防止空包覆盖旧数据
        if (pkg.track_planning_time.size() > 0) {
            // 新轨迹包判定：起始时间变化 -> 触发跳变平滑机制（offset decay）
            if (pkg.track_planning_time[0] != last_traj_start_time_) {
                if (loop_broken) {
                    is_new_traj_received_ = true;
                }
                last_traj_start_time_ = pkg.track_planning_time[0];
            }
            track_data_ = pkg;
            // 第一次收到数据，打印调试信息
            if (!loop_broken) {
                std::cout << "<<<<<<<<<<<<<<<<<<TRAJ--TRACKING START>>>>>>>>>>>>>>>>" << std::endl;
                if(track_data_.track_planning_time.size() > 0)
                    std::cout << "End Time: " << track_data_.track_planning_time.back() << std::endl;
            }
            loop_broken = true; // 标记已收到过数据
            checknum = 0;       // 重置计数器
        }
        // 2. 空包/丢包 处理逻辑 (Wait / Hover)
        // 情况：从未收到过数据(loop_broken=false) 且 当前包为空
        // if (checknum < 1000 && !loop_broken && pkg.track_number == 0) {
            if (checknum < 1000 && !loop_broken ) {
            std::cout << "<<<<<<<<<<<<<<<<<<未收到轨迹包数据>>>>>>>>>>>>>>>> Checknum: " << checknum << std::endl;
            checknum++;
            set_Safe_Hovering(drone_ctrl);
            return 1; // 状态：等待中
        }
        // 3. 超时处理
        if (!loop_broken && checknum >= 1000) {
            set_Safe_Hovering(drone_ctrl);
            traj_finish_flag = true;
            return 2; // 状态：超时/结束
        }
        return 0; // 状态：正常，可以进行计算
    }
    // 根据时间更新参考状态（移植自实机部署版：时间索引 + 二阶插值 + 跳变平滑）
    void Controller_Base::UpdateReferenceFromTrack(double offboard_time, const Eigen::Vector3d& fix_origin)
    {
        if (track_data_.track_number < 2) return;
        // 1. 计算实际控制周期 dt_ctrl（用于 offset 衰减运算），异常时退回 100Hz 默认值
        double dt_ctrl = 0.01;
        if (last_update_time_ > 0.0) {
            dt_ctrl = offboard_time - last_update_time_;
            if (dt_ctrl <= 0.0 || dt_ctrl > 0.1) dt_ctrl = 0.01; // 异常保护
        }
        last_update_time_ = offboard_time;

        // 2. 按 offboard_time 查找当前所在的轨迹段 [countt, countt+1]
        for (countt = 0; countt < track_data_.track_number - 2; countt++) {
            if (offboard_time >= track_data_.track_planning_time[countt] && offboard_time < track_data_.track_planning_time[countt+1]) break;
        }
        if (offboard_time < track_data_.track_planning_time[0]) countt = 0;

        // 3. 提取轨迹段数据
        const double tp = track_data_.track_planning_time[countt];
        const double tq = track_data_.track_planning_time[countt + 1];
        const double dt = tq - tp;

        Pos_p = track_data_.track_planning_p[countt];
        Vel_p = track_data_.track_planning_v[countt];
        Vel_q = track_data_.track_planning_v[countt + 1];
        Acc_p = track_data_.track_planning_a[countt];

        // 首帧特殊处理：用当前反馈状态作参考，避免开局位置误差爆炸
        if (is_first_reference_) {
            Pos_p = state_feedbk.Pos_enu + fix_origin;
            Vel_p = state_feedbk.Vel_enu;
            Vel_q = state_feedbk.Vel_enu;
            is_first_reference_ = false;
        }

        // 只需要计算当前这一段起点的偏移即可，因为参考点是基于 Pos_p 插值得来的
        Pos_p = Pos_p - fix_origin;

        // 4. 计算当前的【原始】期望状态（会发生跳变的值）
        Eigen::Vector3d raw_ref_pos, raw_ref_vel, raw_ref_acc;
        if (std::abs(dt) > 1e-6) {
            double t_diff = offboard_time - tp;
            raw_ref_pos = Pos_p + t_diff * Vel_p + (t_diff * t_diff) / (2.0 * dt) * (Vel_q - Vel_p);
            raw_ref_vel = Vel_p + (t_diff / dt) * (Vel_q - Vel_p);
            raw_ref_acc = Acc_p;
        }
        else {
            raw_ref_pos = Pos_p;
            raw_ref_vel.setZero();
            raw_ref_acc.setZero();
        }

        // =========================================================
        // 5. 跳变平滑处理 (Offset Decay)：新轨迹到来时，新旧参考之间的
        //    断层以时间常数 tau 指数衰减，保证参考状态连续
        // =========================================================
        if (is_new_traj_received_) {
            // 计算当前执行的旧参考位置 与 新轨迹原始位置 之间的断层
            pos_offset_ = state_ref.Pos_enu - raw_ref_pos;
            vel_offset_ = state_ref.Vel_enu - raw_ref_vel;
            is_new_traj_received_ = false; // 清除标志位
        }

        // 衰减时间常数 tau (秒)：越大过渡越平滑，但偏离新规划轨迹的时间越长
        const double tau = 0.6;
        const double decay_factor = std::exp(-dt_ctrl / tau);
        pos_offset_ *= decay_factor;
        vel_offset_ *= decay_factor;

        // 防止极小值持续占用计算资源（浮点数下溢出）
        if (pos_offset_.norm() < 0.01) pos_offset_.setZero();
        if (vel_offset_.norm() < 0.01) vel_offset_.setZero();

        // 6. 最终输出 = 原始插值 + 衰减偏移量
        state_ref.Pos_enu = raw_ref_pos + pos_offset_;
        state_ref.Vel_enu = raw_ref_vel + vel_offset_;
        state_ref.Acc_enu = raw_ref_acc; // 加速度一般前馈直接给，不需要强平滑，否则会破坏动态响应
        // 判断轨迹是否结束
        if (offboard_time > (track_data_.track_planning_time.back() - 0.1)) {
            traj_finish_flag = true;
        }
    }
    //计算旋转矩阵
    void Controller_Base::getRotationMatrix()
    {
        const double phi = state_feedbk.EulerAngles(0);
        const double theta = state_feedbk.EulerAngles(1);
        const double psi = state_feedbk.EulerAngles(2);
        // ZYX: flu -> enu / frd -> ned
        R_b2e <<  cos(psi)*cos(theta) , -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi) , sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi)  ,
                  sin(psi)*cos(theta) , cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi)  , -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi) ,
                  -sin(theta)          , cos(theta)*sin(phi)                             , cos(theta)*cos(phi)                             ;
        // ZYX: enu -> flu / ned -> frd
        R_e2b = R_b2e.transpose();
    }
    //更新反馈状态（状态、旋转矩阵）
    void Controller_Base::SetState_fbk(const State& feedback)
    {
        state_feedbk = feedback;
        // getRotationMatrix();
    }
    //更新参考状态
    void Controller_Base::SetState_ref(const State& ref)
    {
        state_ref = ref;
    }
    //限幅函数
    double Controller_Base::LimitValue(double value, double min_val, double max_val)
    {
        if (value > max_val) return max_val;
        if (value < min_val) return min_val;
        return value;
    }
    //一阶低通滤波
    double Controller_Base::oneorderFilter(double input, double prev_out, double Ts, double tau)
    {
        double alpha = Ts / (tau + Ts);
        return alpha * input + (1.0 - alpha) * prev_out;
    }

// ================= PID 实现 =================
    PIDcontroller::PIDcontroller (double Ts, const PosLoopParam& p, const VelLoopParam& v) : Ts_(Ts), pos_param_(p), vel_param_(v) {}
    void PIDcontroller::Reset()
    {
        vel_err_integral_.setZero();
        prev_vel_err_.setZero();
        control_output = Output{};
    }
    //acc_vel_mode = 1:过载驾驶仪器控制,2:速度控制
    void PIDcontroller::ConductPID(const TrackPackage& pkg, const Eigen::Vector3d& fix_origin, 
        const State& current_fbk, const double offboard_time_, 
        const int acc_vel_mode, FCUControlProtocol& drone_ctrl)
    {
        SetState_fbk(current_fbk);
        int status = CheckTrajectoryStatus(pkg, drone_ctrl);
        if (status == 1) { return;}
        if (status == 2 || traj_finish_flag) {
            traj_finish_flag = true;
            set_Safe_Hovering(drone_ctrl);
            return;
        }
        UpdateReferenceFromTrack(offboard_time_, fix_origin);////////////////////////////////////////////////////
        MapControlToUAV(drone_ctrl, acc_vel_mode);
    }
    // 封装底层赋值、限幅、模式选择
    void PIDcontroller::MapControlToUAV(FCUControlProtocol& drone_ctrl, int mode_type)
    {
        // 公共设置
        drone_ctrl.heading_mode = 1;
        drone_ctrl.attitude_command[2] = CalculateYawCommand(Vel_feedforward, state_feedbk.EulerAngles(2));
        if (mode_type == 1) { 
            // Case: Trajectory_tracking (加速度控制)
            drone_ctrl.horizontal_mode = 4; // 过载/加速度模式
            drone_ctrl.vertical_mode = 4;
            UpdateOutput();
            Eigen::Vector3d final_acc = control_output.Acc_enu;
            drone_ctrl.acceleration_command[0] = LimitValue(final_acc(0), -15.0, 15.0);
            drone_ctrl.acceleration_command[1] = LimitValue(final_acc(1), -15.0, 15.0);
            // 垂向更保守（与实机部署版一致）
            drone_ctrl.acceleration_command[2] = LimitValue(final_acc(2), -7.0, 7.0);
            
        } else if (mode_type == 2) {
            // Case: Landing_tracking (速度控制)
            drone_ctrl.horizontal_mode = 2; // 速度模式
            drone_ctrl.vertical_mode = 2;
            UpdateOutput();
            drone_ctrl.velocity_command[0] = LimitValue(control_output.Vel_enu(0), -22.0f, 22.0f);
            drone_ctrl.velocity_command[1] = LimitValue(control_output.Vel_enu(1), -22.0f, 22.0f);
            drone_ctrl.velocity_command[2] = LimitValue(control_output.Vel_enu(2), -3.0f, 5.0f);
        }
    }
    void PIDcontroller::UpdateOutput()
    {
        PositionLoop();
        VelocityLoop();
    }
    // 运动学平方根控制器实现（ArduPilot 风格）
    double PIDcontroller::SqrtController(double error, double p_gain, double max_accel)
    {
        if (p_gain <= 0.0) return 0.0;

        // 计算线性区与非线性区的临界距离
        double linear_dist = max_accel / (p_gain * p_gain);
        double abs_error = std::abs(error);

        if (abs_error < linear_dist) {
            // 近距离：纯线性 P 控制，保证收敛和平滑
            return p_gain * error;
        } else {
            // 远距离：平方根非线性控制，保证匀减速刹车不超调
            double sign = (error > 0.0) ? 1.0 : -1.0;
            return sign * std::sqrt(2.0 * max_accel * (abs_error - linear_dist / 2.0));
        }
    }
    // 位置环（P + 速度前馈）
    void PIDcontroller::PositionLoop()
    {
        Eigen::Vector3d pos_err = state_ref.Pos_enu - state_feedbk.Pos_enu;
        // 设定无人机水平和垂向的最大物理追踪加速度（根据无人机动力学微调）
        const double max_accel_xy = 8.0;
        const double max_accel_z = 5.0;
        Eigen::Vector3d vel_correction;
        vel_correction(0) = SqrtController(pos_err(0), pos_param_.kp(0), max_accel_xy);
        vel_correction(1) = SqrtController(pos_err(1), pos_param_.kp(1), max_accel_xy);
        vel_correction(2) = SqrtController(pos_err(2), pos_param_.kp(2), max_accel_z);
        // 总速度指令 = 轨迹前馈速度 + 修正速度
        vel_cmd_ = state_ref.Vel_enu + vel_correction;
        // 限速
        vel_cmd_ = vel_cmd_.cwiseMax(-pos_param_.vel_lim).cwiseMin(pos_param_.vel_lim);
        control_output.Pos_enu = state_ref.Pos_enu;//给最终输出赋值：期望位置
        Vel_feedforward = state_ref.Vel_enu;//给前馈速度指令成员赋值：前馈速度信号
        control_output.Vel_enu = vel_cmd_;//给最终输出赋值：期望速度
    }
    // 速度环（PID + 加速度前馈）
    void PIDcontroller::VelocityLoop()
    {
        const Eigen::Vector3d vel_err = vel_cmd_ - state_feedbk.Vel_enu;
        const Eigen::Vector3d dvel_err = (vel_err - prev_vel_err_) / Ts_;
        vel_err_integral_ += vel_err * Ts_;
        // 抗积分饱和
        vel_err_integral_ = vel_err_integral_.cwiseMax(-vel_param_.integral_lim).cwiseMin(vel_param_.integral_lim);
        //有无加速度前馈
        // Eigen::Vector3d acc_cmd =state_ref.Acc_enu + vel_param_.kp.cwiseProduct(vel_err) + vel_param_.ki.cwiseProduct(vel_err_integral_) + vel_param_.kd.cwiseProduct(dvel_err);
        Eigen::Vector3d acc_cmd = vel_param_.kp.cwiseProduct(vel_err) + vel_param_.ki.cwiseProduct(vel_err_integral_) + vel_param_.kd.cwiseProduct(dvel_err);
        control_output.Acc_enu = acc_cmd;//给最终输出赋值：期望加速度
        prev_vel_err_ = vel_err;
    }

    // 角度归一化到 [-pi, pi] 防止数值溢出
    double PIDcontroller::NormalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }

    // 成熟稳定的偏航角指令计算逻辑
    double PIDcontroller::CalculateYawCommand(const Eigen::Vector3d& ref_vel, double current_yaw)
    {
        // 1. 速度死区保护：仅计算水平面上的速度大小
        double speed_2d = std::sqrt(ref_vel(0)*ref_vel(0) + ref_vel(1)*ref_vel(1));
        // 如果速度过小（如 < 0.2 m/s），保持当前机头朝向或上一次的指令，避免原地高频抖动
        if (speed_2d < 0.2f) {
            if (is_first_yaw_) {
                return current_yaw; // 还没开始跟踪时，保持当前朝向
            }
            return prev_yaw_cmd_;   // 运动中停下时，保持最后时刻的朝向指令
        }
        // 2. 计算原始目标航向
        double target_yaw = std::atan2(ref_vel(1), ref_vel(0));
        // 初始化历史指令
        if (is_first_yaw_) {
            prev_yaw_cmd_ = current_yaw;
            is_first_yaw_ = false;
        }
        // 3. 优化的就近旋转逻辑 (Unwrap) 
        // 计算当前目标与上一次指令的最小夹角，避免跨越 180 度时的 360 度大掉头
        double yaw_error = NormalizeAngle(target_yaw - prev_yaw_cmd_);
        double desired_yaw = prev_yaw_cmd_ + yaw_error;
        // 4. 速率限幅保护 (Rate Limiting)
        // 防止轨迹中速度方向瞬间大角度突变导致机身失稳
        double max_yaw_change = yaw_rate_limit_ * Ts_; // 单步最大允许变化量
        desired_yaw = prev_yaw_cmd_ + LimitValue(yaw_error, -max_yaw_change, max_yaw_change);
        // 6. 更新历史状态并返回
        prev_yaw_cmd_ = desired_yaw;
        // 最终输出前再次归一化，确保发送给底层飞控的值在标准范围内
        return NormalizeAngle(desired_yaw); 
    }
}