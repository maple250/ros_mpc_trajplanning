#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "types.h"
namespace mpcc{
class Controller_Base
{
public:
    struct State
    {
        // 时间（循环开始至今）
        double t = 0.0;
        // 位置（ENU）
        Eigen::Vector3d Pos_enu{0.0, 0.0, 0.0};
        // 速度（ENU）
        Eigen::Vector3d Vel_enu{0.0, 0.0, 0.0};
        // 加速度（ENU）
        Eigen::Vector3d Acc_enu{0.0, 0.0, 0.0};
        // 角速度（前左上）
        Eigen::Vector3d Omg_flu{0.0, 0.0, 0.0};
        // 欧拉角（ZYX）
        Eigen::Vector3d EulerAngles{0.0, 0.0, 0.0};// roll, pitch, yaw
    };
    struct Output
    {
        // 位置（ENU）
        Eigen::Vector3d Pos_enu{0.0, 0.0, 0.0};
        // 速度（ENU）
        Eigen::Vector3d Vel_enu{0.0, 0.0, 0.0};
        // 加速度（ENU）
        Eigen::Vector3d Acc_enu{0.0, 0.0, 0.0};
        // 欧拉角（ZYX）
        Eigen::Vector3d EulerAngles{0.0, 0.0, 0.0};// roll, pitch, yaw
        // 角速度（前左上）
        Eigen::Vector3d Omg_flu{0.0, 0.0, 0.0};
        //推力油门
        double Throttle = 0.0;
    };
    State state_feedbk;   // 反馈状态
    State state_ref;      // 参考状态
    Output control_output;// 控制输出
    bool traj_finish_flag = false;// --- 跟踪状态标志位 ---

    // Eigen::Matrix3d R_b2e(Eigen::Matrix3d::Identity());   // body  -> earth
    // Eigen::Matrix3d R_e2b(Eigen::Matrix3d::Identity());   // earth -> body
    Eigen::Matrix3d R_b2e = Eigen::Matrix3d::Identity();   // body  -> earth
    Eigen::Matrix3d R_e2b = Eigen::Matrix3d::Identity();   // earth -> body
    //设置安全的悬停模式
    void set_Safe_Hovering(FCUControlProtocol& drone_ctrl);
    /* ------------------------------------------- 处理轨迹包的函数 ------------------------------------------------*/
    // 返回值：0-正常规划中, 1-等待/悬停中, 2-超时/结束
    int CheckTrajectoryStatus(const TrackPackage& pkg, FCUControlProtocol& drone_ctrl);
    // --- 根据时间查找轨迹点并生成参考状态 ---
    void UpdateReferenceFromTrack(double offboard_time, const Eigen::Vector3d& fix_origin);
    /* -------------------------------------------------------------------------------------------------------------*/
    //计算旋转矩阵
    void getRotationMatrix();
    //更新反馈状态（状态、旋转矩阵）
    void SetState_fbk(const State& feedback);
    //更新参考状态
    void SetState_ref(const State& reference);
    
    // 获取轨迹跟踪中间变量（用于日志记录）
    const Eigen::Vector3d& getPosP() const { return Pos_p; }
    const Eigen::Vector3d& getVelP() const { return Vel_p; }

protected://只能在类定义中使用 对象不能调用
    TrackPackage track_data_; // 存储接收到的轨迹包
    int checknum = 0;          // 空包计数器
    bool loop_broken = false;  // 是否已收到过有效数据
    int countt = 0;            // 轨迹索引
    
    // 轨迹跟踪中间变量
    Eigen::Vector3d Pos_p{0.0, 0.0, 0.0};  // 当前轨迹段位置
    Eigen::Vector3d Vel_p{0.0, 0.0, 0.0};  // 当前轨迹段速度
    Eigen::Vector3d Vel_q{0.0, 0.0, 0.0};  // 当前轨迹段速度
    Eigen::Vector3d Acc_p{0.0, 0.0, 0.0};  // 当前轨迹段加速度

    //限幅函数
    double LimitValue(double value, double min_val, double max_val);
    //一阶低通滤波 
    double oneorderFilter(double input, double prev_out, double Ts, double tau);
};

class PIDcontroller  : public Controller_Base
{
public:
    //PID参数结构体
    struct PosLoopParam
    {
        Eigen::Vector3d kp{1.0, 1.0, 1.0};
        Eigen::Vector3d vel_lim{25.0, 25.0, 25.0};
    };
    struct VelLoopParam
    {
        Eigen::Vector3d kp{1.0, 1.0, 1.0};
        Eigen::Vector3d ki{0.0, 0.0, 0.0};
        Eigen::Vector3d kd{0.0, 0.0, 0.0};
        Eigen::Vector3d integral_lim{10.0, 10.0, 10.0};
    };
    PIDcontroller (double Ts, const PosLoopParam& p, const VelLoopParam& v);
    void Reset();
    void ConductPID(const TrackPackage& pkg, const Eigen::Vector3d& fix_origin, 
        const State& current_fbk, const double offboard_time_, 
        const int acc_vel_mode, FCUControlProtocol& drone_ctrl);
    // mode_type: 1=Trajectory_tracking(加速度控制), 2=Landing(速度控制)
    void MapControlToUAV(FCUControlProtocol& drone_ctrl, int mode_type);
    void UpdateOutput();
private:
    double Ts_;//控制周期
    PosLoopParam pos_param_;
    VelLoopParam vel_param_;
    Eigen::Vector3d vel_cmd_{0.0,0.0,0.0};//速度指令
    Eigen::Vector3d vel_err_integral_{0.0,0.0,0.0};//速度误差积分
    Eigen::Vector3d prev_vel_err_{0,0,0};
    Eigen::Vector3d Vel_feedforward{0.0, 0.0, 0.0};// 速度前馈记录
    // 位置环（P + 速度前馈）
    void PositionLoop();
    // 速度环（PID + 加速度前馈）
    void VelocityLoop();

    // --- 偏航角控制相关变量 ---
    double prev_yaw_cmd_ = 0.0f;       // 上一时刻偏航角指令
    double yaw_rate_limit_ = 1.8f;     // 偏航角速率限制 (rad/s)，约 85度/秒，可根据机动性调整
    bool is_first_yaw_ = true;        // 标志位：是否为第一次计算航向
    
    // --- 偏航角计算相关函数 ---
    // 角度归一化到 [-pi, pi]
    double NormalizeAngle(double angle);
    // 核心航向计算函数
    double CalculateYawCommand(const Eigen::Vector3d& ref_vel, double current_yaw);
    
};}
#endif //CONTROLLER_H