// ============================================================
// 离线拦截仿真主程序（无 ROS/Gazebo/PX4/MAVROS 依赖）
//
// 链路：目标模拟(0.5Hz) → KF → MPC(10Hz) → PID(100Hz)
//       → AccelToAttitudeController（加速度→姿态+推力，替代 PX4 位置/加速度接口）
//       → drone_dynamic 姿态外环 + 角速度内环 + 混控 → 四旋翼动力学（100Hz）
//
// 状态机/三频率节拍/目标机动/logData 日志项与在线版 mpc_planning_node.cpp
// 完全同构，仅把 "MAVROS→PX4→Gazebo" 替换为进程内动力学闭环；
// 12 窗 matplotlib 绘图（MPC::logPlot）与在线版共用同一套代码，模式不变。
//
// 用法：mpc_offline_sim [Params目录] [drone_params.json] [--selftest] [--hover-only]
//   --selftest    : 转换器三断言自检（悬停/前倾/倾角限幅）后退出
//   --hover-only  : 只跑 WAIT→TAKEOFF→悬停5s→阶跃+20m 验证，不接 MPC，不弹窗
// ============================================================
#include <iostream>
#include <array>
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

#include "MPC/mpc.h"
#include "Plotting/plotting.h"
#include "KFfilter/KFfilter.h"
#include "controller/controller.h"

#undef gravity  // mpcc config.h 把 gravity 定义为 9.81 宏，会打坏下面的成员访问与形参名

#include "InnerLoop/AccelToAttitudeController.h"

#include "Model/drone_dynamics.h"
#include "Controller/attitude_controller.h"
#include "Controller/rate_controller.h"
#include "Controller/mixer.h"
#include "Frames/frames.h"

using namespace mpcc;

namespace {
constexpr double kCtrlDt        = 0.01;  // 100Hz 主心跳（与在线 PID 同频、drone_dynamic 默认积分步长）
constexpr int    kTickTarget    = 200;   // 0.5Hz 目标模拟（每 200 拍）
constexpr int    kTickMpc       = 10;    // 10Hz MPC+KF（每 10 拍）
constexpr double kTotalTime     = 60.0;  // 兜底仿真时长 [s]
constexpr double kHoverHoldTime = 10.0;  // EMERGENCY_HOVER 持续时长 [s]
constexpr double kReachDist     = 350.0; // 交接距离（与 MPC::reached_detection 阈值一致）
constexpr double kFcuConnectTime = 0.5;  // 模拟 FCU connected 时刻 [s]
constexpr double kFcuArmTime     = 1.5;  // 模拟 armed+OFFBOARD 时刻 [s]
constexpr double kTakeoffBand   = 0.25;  // 起飞入带阈值 [m]（同在线 L303）
// hover-only 验证：悬停保持与阶跃时机
constexpr double kHoverTestHold = 5.0;
constexpr double kHoverTestStep = 20.0;
constexpr double kHoverTestEnd  = 15.0;
// 电机下标重排：窗口8标签序 M1前右 M2后左 M3前左 M4后右 ← drone_dynamic 1右后 2右前 3左后 4左前
constexpr int kPlotSlotFromDd[4] = {1, 2, 3, 0}; // 槽0←dd[2] 槽1←dd[3] 槽2←dd[4] 槽3←dd[1]
}

//飞行状态机（与在线版一致的四个阶段；DONE 供主循环退出；HOVER_TEST 供 --hover-only）
enum class FlightState {
    WAIT_FOR_CONNECTION,
    TAKEOFF,
    TRACKING_MPC,
    EMERGENCY_HOVER,
    HOVER_TEST,
    DONE
};

//=============== AccelToAttitudeController 自检（三断言） ===============
static int runSelfTest(double gravity) {
    AccelToAttitudeController a2a(gravity);
    int failed = 0;
    auto report = [](const char *name, bool ok, const AttThrustSp &s) {
        std::cout << "[selftest] " << name << ": attSp=(roll " << s.attSp.roll
                  << ", pitch " << s.attSp.pitch << ", yaw " << s.attSp.yaw
                  << ") thrust=" << s.thrustSp << " -> " << (ok ? "PASS" : "FAIL") << std::endl;
        return !ok;
    };
    { // ① a=0 → 水平姿态，推力≈g
        AttThrustSp s = a2a.update(Eigen::Vector3d::Zero(), 0.0);
        bool ok = std::fabs(s.attSp.roll) < 1e-6 && std::fabs(s.attSp.pitch) < 1e-6 &&
                  std::fabs(s.attSp.yaw) < 1e-6 && std::fabs(s.thrustSp - gravity) < 1e-6;
        failed += report("hover(a=0)", ok, s);
    }
    { // ② 前向加速 → FLU 正 pitch（低头，与 drone_validation attLoop 符号一致），推力>g
        AttThrustSp s = a2a.update(Eigen::Vector3d(5.0, 0.0, 0.0), 0.0);
        bool ok = s.attSp.pitch > 0.3 && s.attSp.pitch < 0.7 &&
                  std::fabs(s.attSp.roll) < 1e-6 && s.thrustSp > gravity;
        failed += report("fwd(a=5,0,0)", ok, s);
    }
    { // ③ 大水平指令 → 倾角限幅 60° 生效
        AttThrustSp s = a2a.update(Eigen::Vector3d(100.0, 0.0, 0.0), 0.0);
        bool ok = std::fabs(s.attSp.pitch - drone_dynamic::degToRad(60.0)) < 1e-3;
        failed += report("tilt-limit(a=100)", ok, s);
    }
    return failed;
}

int main(int argc, char** argv) {
    // ---------- 参数与模式解析（无 roslib：编译期宏 + 位置参数覆盖） ----------
    std::string params_dir = MPC_PARAMS_DIR;
    std::string drone_json = DRONE_PARAMS_JSON;
    bool selftest = false, hover_only = false;
    std::vector<std::string> pos_args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if      (a == "--selftest")   selftest = true;
        else if (a == "--hover-only") hover_only = true;
        else pos_args.push_back(a);
    }
    if (!pos_args.empty())    params_dir = pos_args[0];
    if (pos_args.size() > 1)  drone_json = pos_args[1];

    PathToJson json_paths {
        params_dir + "/cost.json",
        params_dir + "/bounds.json",
        params_dir + "/normalization.json",
        params_dir + "/stateInitialization.json"
    };

    // ---------- 算法模块（初始化次序照在线 mpc_planning_node L57-97） ----------
    PIDcontroller::PosLoopParam pos_p;
    pos_p.kp << 0.6, 0.6, 1.0;
    pos_p.vel_lim << 25.0, 25.0, 15.0;
    PIDcontroller::VelLoopParam vel_p;
    vel_p.kp << 0.7, 0.7, 0.7;
    vel_p.ki << 0.05, 0.05, 0.05;
    vel_p.kd << 0.0, 0.0, 0.0;
    vel_p.integral_lim << 5.0, 5.0, 5.0;
    PIDcontroller pid(kCtrlDt, pos_p, vel_p); // 100Hz Ts = 0.01s

    MPC mpc(1, 5, 1.0, Ts, json_paths);

    // 目标初值与运动模式（照在线 L72-86：1 匀速直线；2 匀速圆周 omega=Vh/R）
    Eigen::Vector3d true_target_p = mpc.state_param_.Pos_target_init;
    Eigen::Vector3d true_target_v = mpc.state_param_.Vel_target_init;
    const int target_movetype = mpc.state_param_.Target_movetype;
    double target_turn_rate = 0.0;
    if (target_movetype == 2) {
        if (std::abs(mpc.state_param_.Target_circle_R) < 1e-6) {
            std::cout << "[offline] Circle radius Rt_init is ~0: circular motion degenerates to straight line." << std::endl;
        } else {
            target_turn_rate = mpc.state_param_.Target_vel_h / mpc.state_param_.Target_circle_R;
        }
    } else if (target_movetype != 1) {
        std::cout << "[offline] Target movetype " << target_movetype << " is reserved: treating as straight line." << std::endl;
    }
    const Eigen::Vector3d UAV_p = mpc.state_param_.Pos_self_init; // 起飞目标点（同在线）

    KFfilter kf;
    kf.init(true_target_p, true_target_v);

    // ---------- drone_dynamic 动力学与串级内环 ----------
    drone_dynamic::DroneParams P(drone_json);
    drone_dynamic::DroneDynamics dyn(P.model);
    dyn.setIntegrator(P.integrator.dt, static_cast<drone_dynamic::IntegrateMethod>(P.integrator.method));
    drone_dynamic::AttitudeController att_ctrl(P.attCtrl, P.model.gravity);
    drone_dynamic::RateController rate_ctrl(P.rateCtrl);
    drone_dynamic::Mixer mixer(P.model, P.mixer);
    AccelToAttitudeController accel2att(P.model.gravity);

    if (selftest) {
        return runSelfTest(P.model.gravity) == 0 ? 0 : 1;
    }

    // 初值：悬停配平状态（电机已处悬停转速），从地面出发，起飞到 Pos_self_init（同在线 iris 时序）
    drone_dynamic::DroneState x = dyn.hoverState();
    x.pos.x = 0.0; x.pos.y = 0.0; x.pos.z = 0.0;

    // ---------- 状态机与运行时量 ----------
    FlightState state = FlightState::WAIT_FOR_CONNECTION;
    double start_track_time = 0.0;
    double hover_start_time = 0.0;
    double hover_test_start = 0.0;
    bool   hover_test_stepped = false;
    Eigen::Vector3d hover_pos = Eigen::Vector3d::Zero();
    const Eigen::Vector3d takeoff_pos = UAV_p;

    bool has_new_target_meas = false;
    Eigen::Vector3d meas_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d meas_v = Eigen::Vector3d::Zero();
    TrackPackage shared_track_pkg{};
    shared_track_pkg.track_number = 0;

    Controller_Base::State ego_fbk;
    mpcc::State mpc_ego_state;
    mpc_ego_state.setZero();

    // 100Hz 控制量缓存（供 10Hz logData 采样，同在线 pid_accel_cmd_ 等）
    std::array<double,3> last_pid_accel = {0, 0, 0};
    std::array<double,3> last_ref_pos   = {0, 0, 0};
    std::array<double,3> last_ref_vel   = {0, 0, 0};
    std::array<double,3> last_pid_vel   = {0, 0, 0};

    std::cout << "[offline] Simulation start. Total time limit " << kTotalTime << "s."
              << " (movetype=" << target_movetype << ")" << std::endl;

    int tick = 0;
    for (double t = 0.0; t < kTotalTime && state != FlightState::DONE; t += kCtrlDt, ++tick) {
        // ============ 0.5Hz 目标机动模拟（照在线 targetSensorTimer L203-216，dt=2.0s） ============
        if (state == FlightState::TRACKING_MPC && tick % kTickTarget == 0) {
            constexpr double kTargetDt = 2.0;
            if (target_movetype == 2) {
                Eigen::Matrix3d R_turn = Eigen::AngleAxisd(target_turn_rate * kTargetDt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                true_target_v = R_turn * true_target_v;
            }
            true_target_p += true_target_v * kTargetDt;
            meas_p = true_target_p;
            meas_v = true_target_v;
            has_new_target_meas = true;
        }

        // ============ 反馈状态更新（TRACKING 阶段，照在线 odomCallback） ============
        if (state == FlightState::TRACKING_MPC) {
            ego_fbk.Pos_enu = Eigen::Vector3d(x.pos.x, x.pos.y, x.pos.z);
            ego_fbk.Vel_enu = Eigen::Vector3d(x.vel.vx, x.vel.vy, x.vel.vz);
            const drone_dynamic::EulerAngle att_meas = dyn.getEuler(x);
            ego_fbk.EulerAngles = Eigen::Vector3d(att_meas.roll, att_meas.pitch, att_meas.yaw);
            mpc_ego_state.px = x.pos.x; mpc_ego_state.py = x.pos.y; mpc_ego_state.pz = x.pos.z;
            mpc_ego_state.vx = x.vel.vx; mpc_ego_state.vy = x.vel.vy; mpc_ego_state.vz = x.vel.vz;
        }

        // ============ 10Hz MPC 宏观规划（照在线 mpcPlannerTimer，但不调 reached_detection） ============
        if (state == FlightState::TRACKING_MPC && tick % kTickMpc == 0) {
            const double offboard_time = t - start_track_time;
            // KF 数据融合与演进
            if (has_new_target_meas) {
                kf.update(meas_p, meas_v);
                has_new_target_meas = false;
            }
            kf.predict(0.1);

            TargetState target_kf;
            target_kf.p_t = kf.getPosition();
            target_kf.v_t = kf.getVelocity();
            target_kf.a_t = kf.getAcceleration();

            // 运行 MPC 并记录（8 参数 logData，与在线 L251-252 同构）
            TrackPackage traj_pack = mpc.runInterceptMPC(mpc_ego_state, target_kf, offboard_time);
            // 电机 PWM 真值：归一化转速状态→油门→µs（悬停对应约 1500µs），按下标重排对齐窗口8标签
            std::array<double,4> pwm{};
            for (int s = 0; s < 4; ++s) {
                pwm[s] = 1000.0 + 1000.0 * (x.motor.w[kPlotSlotFromDd[s]] + 1.0) / 2.0;
            }
            mpc.logData(mpc_ego_state, target_kf, traj_pack, pwm, last_pid_accel,
                        last_ref_pos, last_ref_vel, last_pid_vel);
            shared_track_pkg = traj_pack;

            // 触达检测：自算 dist（不调 MPC::reached_detection，其内部会阻塞弹窗，离线版统一退出时画）
            const double dist = (Eigen::Vector3d(x.pos.x, x.pos.y, x.pos.z) - target_kf.p_t).norm();
            const double vel = ego_fbk.Vel_enu.norm();
            std::cout << "Time: " << offboard_time << "s | Dist: " << dist << "m | Vel: " << vel << "m/s" << std::endl;
            if (dist <= kReachDist) {
                std::cout << "[TARGET REACHED] Handing over to terminal guidance. Hovering." << std::endl;
                hover_pos = Eigen::Vector3d(x.pos.x, x.pos.y, x.pos.z);
                hover_start_time = t;
                state = FlightState::EMERGENCY_HOVER;
            }
        }

        // ============ 100Hz 状态机控制律（照在线 controlLoopTimer） ============
        Eigen::Vector3d a_cmd = Eigen::Vector3d::Zero();
        double yaw_sp = 0.0;

        if (state == FlightState::WAIT_FOR_CONNECTION) {
            if (t >= kFcuConnectTime) {
                std::cout << "[offline] FCU connected (simulated). Takeoff." << std::endl;
                state = FlightState::TAKEOFF;
            }
        }
        else if (state == FlightState::TAKEOFF || state == FlightState::EMERGENCY_HOVER || state == FlightState::HOVER_TEST) {
            // 位置P→速度限幅→机动加速度（在线版此阶段发位置 setpoint 给 PX4 位置环，离线版内联实现）
            const Eigen::Vector3d p_des = (state == FlightState::TAKEOFF) ? takeoff_pos : hover_pos;
            const Eigen::Vector3d p(x.pos.x, x.pos.y, x.pos.z);
            const Eigen::Vector3d v(x.vel.vx, x.vel.vy, x.vel.vz);
            Eigen::Vector3d v_des = pos_p.kp.cwiseProduct(p_des - p);
            v_des = v_des.cwiseMax(-pos_p.vel_lim).cwiseMin(pos_p.vel_lim);
            a_cmd = vel_p.kp.cwiseProduct(v_des - v);

            if (state == FlightState::TAKEOFF) {
                // armed+OFFBOARD 后才判定入带（照在线 L303）
                if (t >= kFcuArmTime && std::fabs(x.pos.z - takeoff_pos.z()) < kTakeoffBand) {
                    if (hover_only) {
                        std::cout << "[offline] Takeoff complete. HOVER_TEST begin." << std::endl;
                        hover_pos = takeoff_pos;
                        hover_test_start = t;
                        state = FlightState::HOVER_TEST;
                    } else {
                        std::cout << "[offline] Takeoff complete. Switching to MPC Tracking Mode." << std::endl;
                        start_track_time = t;
                        pid.Reset();
                        rate_ctrl.reset(); // 清姿态内环积分记忆，防切换瞬间污染（对应在线 pid.Reset）
                        state = FlightState::TRACKING_MPC;
                    }
                }
            } else if (state == FlightState::EMERGENCY_HOVER) {
                if (t - hover_start_time >= kHoverHoldTime) state = FlightState::DONE;
            } else { // HOVER_TEST：悬停 5s 后阶跃 +20m，再保持至结束
                if (!hover_test_stepped && t - hover_test_start >= kHoverTestHold) {
                    hover_pos += Eigen::Vector3d(0.0, 0.0, kHoverTestStep);
                    hover_test_stepped = true;
                    std::cout << "[offline] Step +" << kHoverTestStep << "m in z." << std::endl;
                }
                if (t - hover_test_start >= kHoverTestEnd) state = FlightState::DONE;
            }
        }
        else if (state == FlightState::TRACKING_MPC) {
            if (shared_track_pkg.track_number > 0) {
                const double offboard_time = t - start_track_time;
                FCUControlProtocol drone_ctrl;
                // 1 = 加速度控制模式（同在线 L324）
                pid.ConductPID(shared_track_pkg, Eigen::Vector3d::Zero(), ego_fbk, offboard_time, 1, drone_ctrl);
                for (int i = 0; i < 3; ++i) {
                    last_pid_accel[i] = drone_ctrl.acceleration_command[i];
                    last_ref_pos[i]   = pid.state_ref.Pos_enu(i);
                    last_ref_vel[i]   = pid.state_ref.Vel_enu(i);
                    last_pid_vel[i]   = pid.control_output.Vel_enu(i);
                }
                a_cmd  = Eigen::Vector3d(drone_ctrl.acceleration_command[0],
                                         drone_ctrl.acceleration_command[1],
                                         drone_ctrl.acceleration_command[2]);
                yaw_sp = drone_ctrl.attitude_command[2]; // 偏航设定由 PID 生成（对准目标航迹）
            }
        }

        // ============ 内环：加速度→姿态+推力→角速度→混控→动力学（每拍执行） ============
        const AttThrustSp ats = accel2att.update(a_cmd, yaw_sp);
        const drone_dynamic::EulerAngle att_meas = dyn.getEuler(x);
        const drone_dynamic::AttitudeOutput ao = att_ctrl.update(ats.attSp, att_meas); // 只取 rateSp；其 thrustSp 为悬停假设，弃用
        double ang_acc[3];
        rate_ctrl.update(ao.rateSp, x.omega, kCtrlDt, ang_acc);
        const drone_dynamic::MixerOutput mo = mixer.allocate(ats.thrustSp, ang_acc);   // 精确总推力 = ||a_cmd + g·e3||
        x = dyn.step(x, mo.cmd);

        // ============ hover-only 状态打印（1Hz） ============
        if (hover_only && tick % 100 == 0) {
            const drone_dynamic::EulerAngle e = dyn.getEuler(x);
            printf("[hover] t=%5.1fs z=%7.3f att=(%6.2f,%6.2f,%6.2f)deg pwm=(%.0f,%.0f,%.0f,%.0f)\n",
                   t, x.pos.z, drone_dynamic::radToDeg(e.roll), drone_dynamic::radToDeg(e.pitch),
                   drone_dynamic::radToDeg(e.yaw),
                   1000.0 + 500.0 * (x.motor.w[kPlotSlotFromDd[0]] + 1.0),
                   1000.0 + 500.0 * (x.motor.w[kPlotSlotFromDd[1]] + 1.0),
                   1000.0 + 500.0 * (x.motor.w[kPlotSlotFromDd[2]] + 1.0),
                   1000.0 + 500.0 * (x.motor.w[kPlotSlotFromDd[3]] + 1.0));
        }
    }

    std::cout << "[offline] Simulation end at t=" << (tick * kCtrlDt) << "s, state="
              << static_cast<int>(state) << std::endl;

    // ---------- 绘图：退出时统一弹 12 窗（仅一次；hover-only 不画） ----------
    if (!hover_only) {
        std::cout << "[offline] Rendering 12 independent windows. Close ALL windows to exit." << std::endl;
        mpc.logPlot(Ts, json_paths);
    }
    return 0;
}
