#include <iostream>
#include <vector>
#include <cmath>
#include <mutex>
#include <atomic>
#include <array>

#include <ros/ros.h>
#include <ros/package.h>

// MAVROS 消息
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>  // 【新增】解锁服务
#include <mavros_msgs/SetMode.h>      // 【新增】切模式服务
#include <mavros_msgs/RCOut.h>        // 【新增】电机 PWM 反馈（SERVO_OUTPUT_RAW）
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
//=========================================================//
std::string pkg_path = ros::package::getPath("mpcplanning");
std::string base_dir = pkg_path + "/";
PathToJson json_paths {
    base_dir + "Params/cost.json", 
    base_dir + "Params/bounds.json", 
    base_dir + "Params/normalization.json",
    base_dir + "Params/stateInitialization.json"
};
//=========================================================//

class MPCPlanningNode {
public:
    MPCPlanningNode(ros::NodeHandle& nh) : nh_(nh), flight_state_(FlightState::WAIT_FOR_CONNECTION) {
        // --- 1. ROS 发布与订阅 ---
        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &MPCPlanningNode::stateCallback, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, &MPCPlanningNode::odomCallback, this);
        rc_out_sub_ = nh_.subscribe<mavros_msgs::RCOut>("mavros/rc/out", 10, &MPCPlanningNode::rcOutCallback, this);
        
        // MAVROS 控制指令发布 (使用 PositionTarget 支持位置、速度、加速度混合控制)
        setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        // --- 2. 算法模块初始化 ---
        // 初始化 PID 控制器 ,延迟初始化
        PIDcontroller::PosLoopParam pos_p;
        pos_p.kp << 0.6, 0.6, 1.0;
        pos_p.vel_lim << 25.0, 25.0, 15.0;
        PIDcontroller::VelLoopParam vel_p;
        vel_p.kp << 0.7, 0.7, 0.7;
        vel_p.ki << 0.05, 0.05, 0.05;
        vel_p.kd << 0.0, 0.0, 0.0;
        vel_p.integral_lim << 5.0, 5.0, 5.0;

        pid_controller_ = std::make_unique<PIDcontroller>(0.01, pos_p, vel_p); // 100Hz Ts = 0.01s
        //堆分配 + 独占智能指针：可以避开成员变量必须在初始化列表中赋值的要求，允许空指针不做任何事情就度过初始化阶段
        mpc_ = std::make_unique<MPC>(1, 5, 1.0, Ts, json_paths);
        
        true_target_p_ = mpc_->state_param_.Pos_target_init;
        true_target_v_ = mpc_->state_param_.Vel_target_init;
        // 目标运动模式：1 匀速直线；2 匀速圆周，角速度 omega = Vh/R（R 带符号，正为逆时针盘旋）
        target_movetype_ = mpc_->state_param_.Target_movetype;
        if (target_movetype_ == 2) {
            if (std::abs(mpc_->state_param_.Target_circle_R) < 1e-6) {
                ROS_WARN("[Geffen] Circle radius Rt_init is ~0: circular motion degenerates to straight line.");
            } else {
                target_turn_rate_ = mpc_->state_param_.Target_vel_h / mpc_->state_param_.Target_circle_R;
            }
        } else if (target_movetype_ != 1) {
            ROS_WARN("[Geffen] Target movetype %d is reserved and not implemented: treating as straight line.", target_movetype_);
        }
        UAV_p = mpc_->state_param_.Pos_self_init;
        UAV_v = mpc_->state_param_.Vel_self_init;
        
        kf_.init(true_target_p_, true_target_v_);
        // --- 3. 多频定时器初始化 ---
        // 0.5Hz 目标传感器模拟
        timer_0_5hz_ = nh_.createTimer(ros::Duration(2.0), &MPCPlanningNode::targetSensorTimer, this);
        // 10Hz MPC 规划与 KF 滤波
        timer_10hz_ = nh_.createTimer(ros::Duration(0.1), &MPCPlanningNode::mpcPlannerTimer, this);
        // 100Hz PID 底层跟踪与 MAVROS 发布
        timer_100hz_ = nh_.createTimer(ros::Duration(0.01), &MPCPlanningNode::controlLoopTimer, this);

        last_request_time_ = ros::Time::now(); // 初始化请求时间标记
        ROS_INFO("[Geffen] Node Initialized. Waiting for FCU connection and offboard mode...");
    }
    ~MPCPlanningNode() {
        mpc_->logPlot(Ts,json_paths);
        ROS_INFO("=======================================\n");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber rc_out_sub_;
    ros::Publisher setpoint_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    ros::Timer timer_0_5hz_;
    ros::Timer timer_10hz_;
    ros::Timer timer_100hz_;
    //Initialization
    Eigen::Vector3d true_target_p_{1800.0, 1200.0, 300.0};
    Eigen::Vector3d true_target_v_{-15.0, -0.0, -0.0};
    //目标运动特性（来自 stateInitialization.json）
    int target_movetype_ = 1;      //1 匀速直线 2 匀速圆周 3 预留
    double target_turn_rate_ = 0.0; //匀速圆周角速度 omega = Vh/R (rad/s)，正为逆时针
    Eigen::Vector3d UAV_p{0.0, 0.0, 30.0};
    Eigen::Vector3d UAV_v{0.0, 0.0, 0.0};
    //堆分配 + 独占智能指针：可以避开成员变量必须在初始化列表中赋值的要求，允许空指针不做任何事情就度过初始化阶段
    //允许延迟初始化
    std::unique_ptr<MPC> mpc_;
    std::unique_ptr<PIDcontroller> pid_controller_;

    KFfilter kf_;

    // 状态机与时间控制
    std::atomic<FlightState> flight_state_;
    ros::Time start_track_time_;
    ros::Time last_request_time_; // 【新增】用于限制解锁和服务调用的频率
    bool has_takeoff_reference_ = false;
    Eigen::Vector3d hover_pos_;

    // 无人机状态反馈
    mavros_msgs::State current_state_;
    Controller_Base::State ego_fbk_; // 你在 controller.h 定义的反馈状态
    mpcc::State mpc_ego_state_;      // MPC 使用的 6 维状态

    // 跨线程数据保护
    std::mutex data_mutex_;
    TrackPackage shared_track_pkg_;

    bool has_new_target_meas_ = false;
    Eigen::Vector3d meas_p_, meas_v_;

    // 四电机 PWM 反馈（/mavros/rc/out，µs），无反馈时保持全零
    std::array<double,4> motor_pwm_ = {0, 0, 0, 0};

    // 底层 PID 输出的平滑加速度指令（100Hz 更新，随 10Hz 日志采样，用于绘图对比）
    std::array<double,3> pid_accel_cmd_ = {0, 0, 0};

    // 轨迹插值参考状态与 PID 总速度指令（100Hz 更新，随 10Hz 日志采样，用于绘图对比）
    std::array<double,3> pid_ref_pos_ = {0, 0, 0}; // PID 按时刻插值的参考位置 (ENU)
    std::array<double,3> pid_ref_vel_ = {0, 0, 0}; // PID 按时刻插值的参考速度 (ENU)
    std::array<double,3> pid_vel_cmd_ = {0, 0, 0}; // PID 总速度指令 = 前馈 + 位置环修正 (ENU)

    // ================= Callbacks =================
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
        // 如果断开连接或退出 Offboard，触发安全悬停
        if (current_state_.mode != "OFFBOARD" && flight_state_ == FlightState::TRACKING_MPC) {
            ROS_WARN("Offboard mode lost! Engaging emergency hover.");
            flight_state_ = FlightState::EMERGENCY_HOVER;
            hover_pos_ = Eigen::Vector3d(ego_fbk_.Pos_enu(0), ego_fbk_.Pos_enu(1), ego_fbk_.Pos_enu(2));
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 更新 PID 需要的反馈状态
        ego_fbk_.Pos_enu << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        ego_fbk_.Vel_enu << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
        
        // 四元数转欧拉角 (这里简化处理，如有现成库可用 tf)
        double qx = msg->pose.pose.orientation.x; double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z; double qw = msg->pose.pose.orientation.w;
        double yaw = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
        ego_fbk_.EulerAngles << 0.0, 0.0, yaw; // 主要用到 yaw

        // 更新 MPC 需要的状态
        mpc_ego_state_.px = ego_fbk_.Pos_enu(0); mpc_ego_state_.py = ego_fbk_.Pos_enu(1); mpc_ego_state_.pz = ego_fbk_.Pos_enu(2);
        mpc_ego_state_.vx = ego_fbk_.Vel_enu(0); mpc_ego_state_.vy = ego_fbk_.Vel_enu(1); mpc_ego_state_.vz = ego_fbk_.Vel_enu(2);

        if (flight_state_ == FlightState::WAIT_FOR_CONNECTION && current_state_.connected) {
            flight_state_ = FlightState::TAKEOFF;
            last_request_time_ = ros::Time::now(); // 重置计时器
        }
    }

    // ================= 电机 PWM 反馈 =================
    void rcOutCallback(const mavros_msgs::RCOut::ConstPtr& msg) {
        if (msg->channels.size() >= 4) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 4; ++i) motor_pwm_[i] = static_cast<double>(msg->channels[i]);
        }
    }

    // ================= 0.5Hz 目标模拟器 =================
    void targetSensorTimer(const ros::TimerEvent& event) {
        if (flight_state_ != FlightState::TRACKING_MPC) return;
        double dt = 2.0;
        // 目标机动更新：匀速圆周时水平速度矢量绕 z 轴旋转 omega*dt（vz 保持不变），匀速直线时不旋转
        if (target_movetype_ == 2) {
            Eigen::Matrix3d R_turn = Eigen::AngleAxisd(target_turn_rate_ * dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            true_target_v_ = R_turn * true_target_v_;
        }
        true_target_p_ += true_target_v_ * dt;
        std::lock_guard<std::mutex> lock(data_mutex_);
        meas_p_ = true_target_p_;
        meas_v_ = true_target_v_;
        has_new_target_meas_ = true;
    }

    // ================= 10Hz MPC 宏观规划 =================
    void mpcPlannerTimer(const ros::TimerEvent& event) {
        if (flight_state_ != FlightState::TRACKING_MPC) return;
        // 1. 获取时间戳
        double offboard_time = (ros::Time::now() - start_track_time_).toSec();
        // 2. KF 数据融合与演进
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (has_new_target_meas_) {
                kf_.update(meas_p_, meas_v_);
                has_new_target_meas_ = false; 
            }
        }
        kf_.predict(0.1); 

        TargetState target_kf;
        target_kf.p_t = kf_.getPosition();
        target_kf.v_t = kf_.getVelocity();
        target_kf.a_t = kf_.getAcceleration(); 

        // 3. 运行 MPC
        TrackPackage traj_pack = mpc_->runInterceptMPC(mpc_ego_state_, target_kf, offboard_time);
        // 记录数据用于后续分析和绘图（含电机 PWM 反馈与 PID 平滑加速度指令）
        std::array<double,4> pwm_copy;
        std::array<double,3> pid_accel_copy, ref_pos_copy, ref_vel_copy, pid_vel_copy;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            pwm_copy = motor_pwm_;
            pid_accel_copy = pid_accel_cmd_;
            ref_pos_copy = pid_ref_pos_;
            ref_vel_copy = pid_ref_vel_;
            pid_vel_copy = pid_vel_cmd_;
        }
        mpc_->logData(mpc_ego_state_, target_kf, traj_pack, pwm_copy, pid_accel_copy,
                      ref_pos_copy, ref_vel_copy, pid_vel_copy);
        // 4. 将解算出的轨迹压入共享内存
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            shared_track_pkg_ = traj_pack;
        }

        // 5. 检查是否抵达交接距离
        mpc_->reached_detection(mpc_ego_state_, target_kf, offboard_time,json_paths);
        if(mpc_->traj_finish) {
            ROS_INFO("[TARGET REACHED] Handing over to terminal guidance. Hovering.");
            hover_pos_ = Eigen::Vector3d(mpc_ego_state_.px, mpc_ego_state_.py, mpc_ego_state_.pz);
            flight_state_ = FlightState::EMERGENCY_HOVER;
        }
    }

    // ================= 100Hz PID 底层控制 =================
    void controlLoopTimer(const ros::TimerEvent& event) {
        if (flight_state_ == FlightState::WAIT_FOR_CONNECTION) return;
        mavros_msgs::PositionTarget setpoint_msg;
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // MAVROS 内部映射为 ENU

        if (flight_state_ == FlightState::TAKEOFF) {
            // 起飞阶段：位置控制
            setpoint_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            setpoint_msg.position.x = UAV_p.x();
            setpoint_msg.position.y = UAV_p.y();
            setpoint_msg.position.z = UAV_p.z();
            setpoint_msg.yaw = 0.0;

        // 【核心唤醒逻辑】必须在持续发送 setpoint 的同时，请求切模式和解锁 (频率限制在 1Hz 以免堵塞通信)
            if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_time_ > ros::Duration(1.0))) {
                mavros_msgs::SetMode offb_set_mode;
                offb_set_mode.request.custom_mode = "OFFBOARD";
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard mode enabled");
                }
                last_request_time_ = ros::Time::now();
            } else if (!current_state_.armed && current_state_.mode == "OFFBOARD" && (ros::Time::now() - last_request_time_ > ros::Duration(1.0))) {
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = true;
                if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request_time_ = ros::Time::now();
            }

            // 检查是否抵达高度 (必须在已经解锁且进入 OFFBOARD 之后才开始判定高度)
            if (current_state_.mode == "OFFBOARD" && current_state_.armed && std::abs(ego_fbk_.Pos_enu(2) - UAV_p.z()) < 0.25) {
                ROS_INFO("Takeoff complete. Switching to MPC Tracking Mode.");
                start_track_time_ = ros::Time::now(); 
                pid_controller_->Reset(); 
                flight_state_ = FlightState::TRACKING_MPC;
            }
        } 
        else if (flight_state_ == FlightState::TRACKING_MPC) {
            // 跟踪阶段：提取最新轨迹，执行 100Hz 自身插值 PID
            TrackPackage current_pkg;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                current_pkg = shared_track_pkg_;
            }

            if (current_pkg.track_number > 0) {
                double offboard_time = (ros::Time::now() - start_track_time_).toSec();
                FCUControlProtocol drone_ctrl; // 假设这是你在 guidance_structs 里的通信结构体
                Eigen::Vector3d fix_origin = Eigen::Vector3d::Zero();

                // 传入 1 = 加速度控制模式
                pid_controller_->ConductPID(current_pkg, fix_origin, ego_fbk_, offboard_time, 1, drone_ctrl);

                // 记录 PID 平滑后的加速度指令（MPC 指令与实际响应的中间量，用于绘图对比）
                // 同时记录轨迹插值参考状态与 PID 总速度指令（读取 ConductPID 刚更新的内部量，同线程无竞争）
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    for (int i = 0; i < 3; ++i) {
                        pid_accel_cmd_[i] = drone_ctrl.acceleration_command[i];
                        pid_ref_pos_[i] = pid_controller_->state_ref.Pos_enu(i);
                        pid_ref_vel_[i] = pid_controller_->state_ref.Vel_enu(i);
                        pid_vel_cmd_[i] = pid_controller_->control_output.Vel_enu(i);
                    }
                }

                // 将 FCUControlProtocol 映射给 MAVROS 的加速度控制类型
                // 注意掩码：忽略位置、速度，仅保留加速度和偏航角
                setpoint_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ |
                                         mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |
                                         mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
                
                // MAVROS 要求传入的 Acc 是包含抵消重力推力的指令吗？
                // 通常 MAVROS setpoint_raw 加速度指令是单纯的机动加速度，底层飞控会自己加重力。请以你实际 PX4 参数配置为准。
                setpoint_msg.acceleration_or_force.x = drone_ctrl.acceleration_command[0];
                setpoint_msg.acceleration_or_force.y = drone_ctrl.acceleration_command[1];
                setpoint_msg.acceleration_or_force.z = drone_ctrl.acceleration_command[2];
                setpoint_msg.yaw = drone_ctrl.attitude_command[2];
            }
        }
        else if (flight_state_ == FlightState::EMERGENCY_HOVER) {
            // 悬停阶段：位置控制，锁死在 hover_pos_
            setpoint_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            setpoint_msg.position.x = hover_pos_.x();
            setpoint_msg.position.y = hover_pos_.y();
            setpoint_msg.position.z = hover_pos_.z();
        }

        // 发送给飞控
        // 只要不是断开连接状态，无论飞机当前是不是 Offboard，都必须向底层灌输 Setpoint 数据流，否则飞控会拒绝切换。
        setpoint_pub_.publish(setpoint_msg);

    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpcc_controller_node");
    ros::NodeHandle nh;

    try {
        MPCPlanningNode node(nh);
        // 使用多线程 Spinner 保证 100Hz 计时器不被 10Hz 的计算阻塞
        ros::AsyncSpinner spinner(3); 
        spinner.start();
        ros::waitForShutdown();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Fatal exception: " << e.what());
        return -1;
    }

    return 0;
}