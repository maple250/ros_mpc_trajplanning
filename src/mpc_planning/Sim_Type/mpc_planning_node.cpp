#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>

#include <ros/ros.h>
#include <ros/package.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "MPC/mpc.h"
#include "Plotting/plotting.h"
#include "KFfilter/KFfilter.h"
#include "controller/controller.h"

using namespace mpcc;
// 飞行状态机枚举
enum class FlightState {
    WAIT_FOR_CONNECTION,
    TAKEOFF,
    TRACKING_MPC,
    EMERGENCY_HOVER
};
// 全局变量用于跨线程通信 (模拟传感器数据接收)
std::mutex target_data_mutex;
bool has_new_target_meas = false;
Eigen::Vector3d meas_p;
Eigen::Vector3d meas_v;
// 线程退出标志
std::atomic<bool> simulation_running(true);

// --- 0.5Hz 目标模拟线程 (模拟外部传感器或数据链) ---
void targetSensorThread() {
    ros::Rate slow_rate(0.5); // 严格 0.5 Hz
    double t = 0.0;
    // 目标初始参数
    Eigen::Vector3d true_p(1800.0, 1200.0, 300.0);
    Eigen::Vector3d true_v(-18.0, -15.0, -2.0);
    Eigen::Matrix3d R_turn = Eigen::Matrix3d::Identity();
    while (simulation_running && ros::ok()) {
        // 模拟目标做 XY 盘旋 + Z 轴起伏机动
        double dt = 2.0; // 0.5Hz 周期为 2 秒
        // R_turn = Eigen::AngleAxisd(0.01 * dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        true_v = R_turn * true_v;
        // true_v.z() = 5.0 * std::cos(0.2 * t); // Z轴扰动
        true_p += true_v * dt;
        t += dt;
        {
            std::lock_guard<std::mutex> lock(target_data_mutex);
            meas_p = true_p;
            meas_v = true_v;
            has_new_target_meas = true;
        }
        std::cout << "[Sensor Thread] Received 0.5Hz Target Update." << std::endl;
        slow_rate.sleep(); // 阻塞 2 秒
    }
}

// 模拟真实世界的无人机物理模型 (带阻力)
State simulateDroneDynamics(const State& current_state, const Eigen::Vector3d& cmd_acc, double Ts) {
    State next_state = current_state;
    double drag_coeff = 0.2; // 必须与 mpc.cpp 中硬编码的阻力系数一致
    // 速度更新: v_{k+1} = v_k + (a_k - D * v_k) * Ts
    next_state.vx += (cmd_acc.x() - drag_coeff * current_state.vx) * Ts;
    next_state.vy += (cmd_acc.y() - drag_coeff * current_state.vy) * Ts;
    next_state.vz += (cmd_acc.z() - drag_coeff * current_state.vz) * Ts;
    // 位置更新: p_{k+1} = p_k + v_k * Ts
    next_state.px += current_state.vx * Ts;
    next_state.py += current_state.vy * Ts;
    next_state.pz += current_state.vz * Ts;
    return next_state;
}

int main(int argc, char** argv) {
    //====================1.加载json参数路径=====================//
    ros::init(argc, argv, "mpc_planning_test_node");
    ros::NodeHandle nh;
    std::string pkg_path = ros::package::getPath("mpcplanning");
    if (pkg_path.empty()) 
        std::cerr << "[Geffen Fatal Error] ROS package 'mpcc' not found! Did you source devel/setup.bash?" << std::endl;
    std::string base_dir = pkg_path + "/";
    PathToJson json_paths {
        base_dir + "Params/cost.json", 
        base_dir + "Params/bounds.json", 
        base_dir + "Params/normalization.json"
    };
    //=======================2.初始化MPC========================//
    MPC mpc(1, 5, 1.0, 0.1, json_paths); // n_sqp n_reset sqp_mixing Ts PathToJson
    //====================3.初始化KF滤波器=====================//
    KFfilter kf;
    kf.init(Eigen::Vector3d(1800.0, 1200.0, 300.0), Eigen::Vector3d(-10.0, -10.0, 0.0));
    // 启动 0.5Hz 传感器模拟线程
    std::thread sensor_thread(targetSensorThread);
    // 我方状态初始化
    State ego_state = {0.0, 0.0, 50.0,  0.0, 0.0, 0.0};
    // 严格设定 MPC 主循环频率 10Hz
    double Ts = 0.1; 
    ros::Rate mpc_rate(10.0); 
    double offboard_time = 0.0;

    while (offboard_time < 60.0 && ros::ok()) {
        // --- 1. KF 数据更新与预测 ---
        {
            std::lock_guard<std::mutex> lock(target_data_mutex);
            if (has_new_target_meas) {
                // 收到 0.5Hz 新数据，执行 Update
                kf.update(meas_p, meas_v);
                has_new_target_meas = false; 
            }
        }
        // 无论有无新数据，KF 始终以 10Hz 进行高频状态演进
        kf.predict(Ts); 
        TargetState target_kf;
        target_kf.p_t = kf.getPosition();
        target_kf.v_t = kf.getVelocity();
        target_kf.a_t = kf.getAcceleration(); // KF推算出来的加速度
        // --- 2. 运行宏观拦截 MPC ---
        TrackPackage traj_pack = mpc.runInterceptMPC(ego_state, target_kf, offboard_time);
        mpc.logData(ego_state, target_kf, traj_pack); // 记录数据用于后续分析和绘图
        // --- 3. 物理环境模拟 ---
        ego_state = simulateDroneDynamics(ego_state, traj_pack.track_planning_a[3], Ts);
        offboard_time += Ts;
        // 数据监控
        mpc.reached_detection(ego_state, target_kf, offboard_time);
        if(mpc.traj_finish) break;
        // 强制阻塞，保证 10Hz 严格运行
        mpc_rate.sleep();
    }

    simulation_running = false;
    if(sensor_thread.joinable()) sensor_thread.join();
    
    mpc.logPlot(Ts, json_paths);
    return 0;
}