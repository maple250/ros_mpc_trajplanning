#include "plotting.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace mpcc {

Plotting::Plotting(double Ts, const PathToJson& path) : Ts_(Ts)
{
}

namespace {
// ==========================================
// matplotlib >= 3.6 兼容的 3D 绘制辅助
// matplotlib-cpp 自带的 plot3 使用 fig.gca(projection='3d')，
// matplotlib 3.6 起 Figure.gca() 不再接受 projection 关键字会抛 "No axis"
// （旧版本上其 figure 编号处理还会在 figure_size 之后多开一个图）。
// 这里改为：取当前 figure，若无坐标系则 add_subplot(projection='3d') 新建，
// 否则复用第一个坐标系；绘制后用 pyplot.sca() 将其设为当前 axes，
// 使 plt::legend()/plt::title()/plt::grid() 等能正常作用于 3D 图。
// 兼容容器内 Noetic(matplotlib 3.1) 与宿主机(matplotlib 3.6+)。
void plot3_compat(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z,
                  const std::map<std::string, std::string>& keywords = {})
{
    matplotlibcpp::detail::_interpreter::get(); // 确保 Python 解释器已初始化

    // 惰性加载 3D 工具箱
    static PyObject* axis3dmod = nullptr;
    if (!axis3dmod) {
        PyObject* name = PyUnicode_FromString("mpl_toolkits.mplot3d");
        axis3dmod = PyImport_Import(name);
        Py_DECREF(name);
        if (!axis3dmod) throw std::runtime_error("Could not load mpl_toolkits.mplot3d!");
    }

    // 注意：matplotlibcpp 的 get_array 用 PyArray_SimpleNewFromData 零拷贝直接引用
    // vector 内存。若调用方传入临时 vector（如单点标记 {ego_x.front()}），返回的
    // numpy 数组在完整表达式结束后即悬垂，3D 绘制路径上会读到垃圾数据（标记错位）。
    // 这里统一深拷贝为数组自有内存，消除对 vector 生命周期的依赖。
    auto own_array = [](const std::vector<double>& v) {
        PyObject* zero_copy = matplotlibcpp::detail::get_array(v);
        PyObject* owned = PyArray_NewCopy(reinterpret_cast<PyArrayObject*>(zero_copy), NPY_ANYORDER);
        Py_DECREF(zero_copy);
        return owned;
    };

    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, own_array(x));
    PyTuple_SetItem(args, 1, own_array(y));
    PyTuple_SetItem(args, 2, own_array(z));

    PyObject* kwargs = PyDict_New();
    for (const auto& kw : keywords)
        PyDict_SetItemString(kwargs, kw.first.c_str(), PyUnicode_FromString(kw.second.c_str()));

    // 取当前 figure：无坐标系则新建 3D 坐标系，否则复用第一个
    PyObject* pyplot = PyImport_ImportModule("matplotlib.pyplot");
    PyObject* gcf = PyObject_GetAttrString(pyplot, "gcf");
    PyObject* fig = PyObject_CallObject(gcf, nullptr);
    PyObject* axes_list = PyObject_GetAttrString(fig, "axes");
    PyObject* axis = nullptr;
    if (axes_list && PySequence_Size(axes_list) > 0) {
        axis = PySequence_GetItem(axes_list, 0);
    } else {
        PyObject* add_subplot = PyObject_GetAttrString(fig, "add_subplot");
        PyObject* as_kwargs = PyDict_New();
        PyDict_SetItemString(as_kwargs, "projection", PyUnicode_FromString("3d"));
        PyObject* empty_args = PyTuple_New(0); // PyObject_Call 的 args 不允许为 NULL
        axis = PyObject_Call(add_subplot, empty_args, as_kwargs);
    }
    if (!axis) throw std::runtime_error("Could not create 3D axis!");

    PyObject* plot = PyObject_GetAttrString(axis, "plot");
    if (!plot) throw std::runtime_error("3D axis has no plot method!");
    PyObject* res = PyObject_Call(plot, args, kwargs);
    if (!res) throw std::runtime_error("3D line plot failed!");

    PyObject* sca = PyObject_GetAttrString(pyplot, "sca");
    PyObject* sca_args = PyTuple_New(1);
    PyTuple_SetItem(sca_args, 0, axis);
    PyObject_CallObject(sca, sca_args);
}

// matplotlib-cpp 没有 zlabel，补一个（作用于当前 figure 的第一个坐标系）
void zlabel_compat(const std::string& label)
{
    PyObject* pyplot = PyImport_ImportModule("matplotlib.pyplot");
    PyObject* gcf = PyObject_GetAttrString(pyplot, "gcf");
    PyObject* fig = PyObject_CallObject(gcf, nullptr);
    PyObject* axes_list = PyObject_GetAttrString(fig, "axes");
    if (axes_list && PySequence_Size(axes_list) > 0) {
        PyObject* axis = PySequence_GetItem(axes_list, 0);
        PyObject* set_zlabel = PyObject_GetAttrString(axis, "set_zlabel");
        PyObject* args = PyTuple_New(1);
        PyTuple_SetItem(args, 0, PyUnicode_FromString(label.c_str()));
        PyObject_CallObject(set_zlabel, args);
    }
}
} // anonymous namespace

void Plotting::plotIntercept(const std::vector<State>& ego_log,
                             const std::vector<TargetState>& target_log,
                             const std::vector<TrackPackage>& plan_log,
                             const std::vector<std::array<double,4>>& motor_log,
                             bool motor_pwm_is_real,
                             const std::vector<std::array<double,3>>& mpc_accel_log,
                             const std::vector<std::array<double,3>>& pid_accel_log) const
{
    if (ego_log.empty()) return;

    std::vector<double> ego_x, ego_y, ego_z, ego_v;
    std::vector<double> target_x, target_y, target_z;
    std::vector<double> dist, time;

    double t = 0.0;
    for (size_t i = 0; i < ego_log.size(); ++i) {
        ego_x.push_back(ego_log[i].px);
        ego_y.push_back(ego_log[i].py);
        ego_z.push_back(ego_log[i].pz);

        double v = std::sqrt(std::pow(ego_log[i].vx, 2) + std::pow(ego_log[i].vy, 2) + std::pow(ego_log[i].vz, 2));
        ego_v.push_back(v);

        if (i < target_log.size()) {
            target_x.push_back(target_log[i].p_t.x());
            target_y.push_back(target_log[i].p_t.y());
            target_z.push_back(target_log[i].p_t.z());

            double d = std::sqrt(std::pow(ego_log[i].px - target_log[i].p_t.x(), 2) +
                                 std::pow(ego_log[i].py - target_log[i].p_t.y(), 2) +
                                 std::pow(ego_log[i].pz - target_log[i].p_t.z(), 2));
            dist.push_back(d);
        }
        time.push_back(t);
        t += Ts_; // 由 MPC 实际采样周期决定时间轴
    }

    // ==========================================
    // 窗口 1: XY 平面视图
    // ==========================================
    plt::figure_size(600, 600);
    plt::plot(ego_x, ego_y, {{"label", "Ego Traj"}, {"color", "blue"}, {"linewidth", "2"}});
    plt::plot(target_x, target_y, {{"label", "Target Traj"}, {"color", "red"}, {"linewidth", "2"}});
    plt::plot({ego_x.front()}, {ego_y.front()}, {{"label", "Start Point"}, {"marker", "*"}, {"color", "black"}, {"markersize", "10"}});
    plt::plot({target_x.front()}, {target_y.front()}, {{"marker", "*"}, {"color", "black"}, {"markersize", "10"}});
    // 满足拦截/交接判定条件时刻的敌我最终位置
    plt::plot({ego_x.back()}, {ego_y.back()}, {{"label", "Ego End (Intercept)"}, {"marker", "D"}, {"color", "blue"}, {"markersize", "10"}});
    plt::plot({target_x.back()}, {target_y.back()}, {{"label", "Target End (Intercept)"}, {"marker", "D"}, {"color", "red"}, {"markersize", "10"}});
    int count = 0;
    for (const auto& plan : plan_log) {
        std::vector<double> px, py;
        for (const auto& p : plan.track_planning_p) { px.push_back(p.x()); py.push_back(p.y()); }

        if (count == 0) {
            plt::plot(px, py, {{"label", "MPC Plan"}, {"color", "green"}, {"linestyle", "--"}, {"linewidth", "2"}});
            plt::plot({px.front()}, {py.front()}, {{"label", "Plan Start"}, {"marker", "x"}, {"color", "darkgreen"}, {"markersize", "5"}});
        } else {
            plt::plot(px, py, {{"color", "green"}, {"linestyle", "--"}, {"linewidth", "2"}});
            plt::plot({px.front()}, {py.front()}, {{"marker", "x"}, {"color", "darkgreen"}, {"markersize", "5"}});
        }
        count++;
    }
    plt::title("Window 1: XY Plane (Top-Down)");
    plt::xlabel("X [m]"); plt::ylabel("Y [m]");
    plt::legend(); plt::grid(true); plt::axis("equal");

    // ==========================================
    // 窗口 2: XZ 平面视图 (侧视图)
    // ==========================================
    plt::figure_size(600, 600);
    plt::plot(ego_x, ego_z, {{"label", "Ego Traj"}, {"color", "blue"}, {"linewidth", "2"}});
    plt::plot(target_x, target_z, {{"label", "Target Traj"}, {"color", "red"}, {"linewidth", "2"}});
    plt::plot({ego_x.front()}, {ego_z.front()}, {{"marker", "*"}, {"color", "black"}, {"markersize", "10"}});
    plt::plot({target_x.front()}, {target_z.front()}, {{"marker", "*"}, {"color", "black"}, {"markersize", "10"}});
    count = 0;
    for (const auto& plan : plan_log) {
        std::vector<double> px, pz;
        for (const auto& p : plan.track_planning_p) { px.push_back(p.x()); pz.push_back(p.z()); }

        if (count == 0) plt::plot(px, pz, {{"label", "MPC Plan"}, {"color", "green"}, {"linestyle", "--"}, {"linewidth", "2"}});
        else plt::plot(px, pz, {{"color", "green"}, {"linestyle", "--"}, {"linewidth", "2"}});

        // 标记规划起点
        plt::plot({px.front()}, {pz.front()}, {{"marker", "x"}, {"color", "darkgreen"}, {"markersize", "5"}});
        count++;
    }
    plt::title("Window 2: XZ Plane (Side View)");
    plt::xlabel("X [m]"); plt::ylabel("Z [m]");
    plt::legend(); plt::grid(true); plt::axis("equal");

    // ==========================================
    // 窗口 3: YZ 平面视图 (后视图)
    // ==========================================
    plt::figure_size(600, 600);
    plt::plot(ego_y, ego_z, {{"label", "Ego Traj"}, {"color", "blue"}, {"linewidth", "2"}});
    plt::plot(target_y, target_z, {{"label", "Target Traj"}, {"color", "red"}, {"linewidth", "2"}});
    plt::plot({ego_y.front()}, {ego_z.front()}, {{"marker", "*"}, {"color", "black"}, {"markersize", "10"}});
    plt::plot({target_y.front()}, {target_z.front()}, {{"marker", "*"}, {"color", "black"}, {"markersize", "10"}});
    count = 0;
    for (const auto& plan : plan_log) {
        std::vector<double> py, pz;
        for (const auto& p : plan.track_planning_p) { py.push_back(p.y()); pz.push_back(p.z()); }

        if (count == 0) plt::plot(py, pz, {{"label", "MPC Plan"}, {"color", "green"}, {"linestyle", "--"}, {"linewidth", "2"}});
        else plt::plot(py, pz, {{"color", "green"}, {"linestyle", "--"}, {"linewidth", "2"}});

        // 标记规划起点
        plt::plot({py.front()}, {pz.front()}, {{"marker", "x"}, {"color", "darkgreen"}, {"markersize", "5"}});
        count++;
    }
    plt::title("Window 3: YZ Plane (Rear View)");
    plt::xlabel("Y [m]"); plt::ylabel("Z [m]");
    plt::legend(); plt::grid(true); plt::axis("equal");

    // ==========================================
    // 窗口 4: 性能指标视图
    // ==========================================
    plt::figure_size(800, 300);
    plt::plot(time, dist, {{"label", "Relative Distance [m]"}, {"color", "black"}, {"linewidth", "2"}});
    plt::plot(time, ego_v, {{"label", "Ego Velocity [m/s]"}, {"color", "blue"}, {"linestyle", "-."}, {"linewidth", "2"}});
    std::vector<double> handover_line(time.size(), 350.0);
    plt::plot(time, handover_line, {{"label", "Handover Distance (350m)"}, {"color", "red"}, {"linestyle", ":"}});

    plt::title("Window 4: Engagement Metrics over Time");
    plt::xlabel("Time [s]"); plt::ylabel("Metrics Value");
    plt::legend(); plt::grid(true);

    // ==========================================
    // 窗口 5: 敌我三维轨迹视图
    // ==========================================
    plt::figure_size(700, 600);
    plot3_compat(ego_x, ego_y, ego_z, {{"label", "Ego Traj (3D)"}, {"color", "blue"}, {"linewidth", "2"}});
    plot3_compat(target_x, target_y, target_z, {{"label", "Target Traj (3D)"}, {"color", "red"}, {"linewidth", "2"}});
    // 起飞点 / 结束点
    plot3_compat({ego_x.front()}, {ego_y.front()}, {ego_z.front()},
                 {{"label", "Start Point"}, {"marker", "*"}, {"color", "black"}, {"markersize", "10"}});
    plot3_compat({target_x.front()}, {target_y.front()}, {target_z.front()},
                 {{"marker", "*"}, {"color", "black"}, {"markersize", "10"}});
    plot3_compat({ego_x.back()}, {ego_y.back()}, {ego_z.back()},
                 {{"label", "Ego End (Intercept)"}, {"marker", "D"}, {"color", "blue"}, {"markersize", "8"}});
    plot3_compat({target_x.back()}, {target_y.back()}, {target_z.back()},
                 {{"label", "Target End (Intercept)"}, {"marker", "D"}, {"color", "red"}, {"markersize", "8"}});
    // 间隔时间的 MPC 预测轨迹段
    count = 0;
    for (const auto& plan : plan_log) {
        std::vector<double> px, py, pz;
        for (const auto& p : plan.track_planning_p) { px.push_back(p.x()); py.push_back(p.y()); pz.push_back(p.z()); }

        if (count == 0) {
            plot3_compat(px, py, pz, {{"label", "MPC Plan"}, {"color", "green"}, {"linestyle", "--"}, {"linewidth", "2"}});
            plot3_compat({px.front()}, {py.front()}, {pz.front()},
                         {{"label", "Plan Start"}, {"marker", "x"}, {"color", "darkgreen"}, {"markersize", "6"}});
        } else {
            plot3_compat(px, py, pz, {{"color", "green"}, {"linestyle", "--"}, {"linewidth", "2"}});
            plot3_compat({px.front()}, {py.front()}, {pz.front()},
                         {{"marker", "x"}, {"color", "darkgreen"}, {"markersize", "6"}});
        }
        count++;
    }
    plt::title("Window 5: 3D Intercept Trajectories");
    plt::xlabel("X [m]"); plt::ylabel("Y [m]"); zlabel_compat("Z [m]");
    plt::legend(); plt::grid(true);

    // ==========================================
    // 窗口 6: 敌我相对距离随时间变化
    // ==========================================
    if (!dist.empty()) {
        plt::figure_size(800, 300);
        std::vector<double> time_d(time.begin(), time.begin() + dist.size());
        plt::plot(time_d, dist, {{"label", "Relative Distance"}, {"color", "black"}, {"linewidth", "2"}});
        std::vector<double> handover_line_d(time_d.size(), 350.0);
        plt::plot(time_d, handover_line_d, {{"label", "Handover Distance (350m)"}, {"color", "red"}, {"linestyle", ":"}});
        plt::plot({time_d.back()}, {dist.back()}, {{"label", "End (Intercept)"}, {"marker", "D"}, {"color", "purple"}, {"markersize", "8"}});

        plt::title("Window 6: Relative Distance vs Time");
        plt::xlabel("Time [s]"); plt::ylabel("Relative Distance [m]");
        plt::legend(); plt::grid(true);

        // ==========================================
        // 窗口 7: 我方合速度随敌我相对距离变化
        // ==========================================
        plt::figure_size(800, 400);
        std::vector<double> ego_v_d(ego_v.begin(), ego_v.begin() + dist.size());
        plt::plot(dist, ego_v_d, {{"label", "Ego Speed |v|"}, {"color", "blue"}, {"linewidth", "2"}});

        plt::title("Window 7: Ego Speed vs Relative Distance");
        plt::xlabel("Relative Distance [m]"); plt::ylabel("Ego Speed |v| [m/s]");
        plt::legend(); plt::grid(true);
    }

    // ==========================================
    // 窗口 8: 四电机 PWM 响应
    // ==========================================
    if (!motor_log.empty()) {
        plt::figure_size(800, 400);
        size_t n = std::min(motor_log.size(), time.size());
        std::vector<double> time_m(time.begin(), time.begin() + n);
        const char* motor_names[4] = {"Motor 1 (FR)", "Motor 2 (RL)", "Motor 3 (FL)", "Motor 4 (RR)"};
        const char* motor_colors[4] = {"tab:blue", "tab:orange", "tab:green", "tab:red"};
        for (int m = 0; m < 4; ++m) {
            std::vector<double> pwm_m;
            pwm_m.reserve(n);
            for (size_t i = 0; i < n; ++i) pwm_m.push_back(motor_log[i][m]);
            plt::plot(time_m, pwm_m, {{"label", motor_names[m]}, {"color", motor_colors[m]}, {"linewidth", "1.5"}});
        }

        plt::title(std::string("Window 8: Motor PWM vs Time (") +
                   (motor_pwm_is_real ? "feedback: /mavros/rc/out" : "estimated from MPC accel command") + ")");
        plt::xlabel("Time [s]"); plt::ylabel("Motor PWM [us]");
        plt::legend(); plt::grid(true);
    }

    // ==========================================
    // 窗口 9: 加速度指令链对比
    // MPC 指令 vs 底层 PID 平滑指令 vs 实际加速度响应（速度差分）
    // ==========================================
    if (!mpc_accel_log.empty()) {
        plt::figure_size(800, 750);
        size_t n = std::min(mpc_accel_log.size(), time.size());
        std::vector<double> time_a(time.begin(), time.begin() + n);

        // 实际加速度响应：由速度反馈差分 (v[k]-v[k-1])/Ts 得到
        std::vector<double> acc_resp[3], time_resp;
        for (size_t i = 1; i < ego_log.size(); ++i) {
            acc_resp[0].push_back((ego_log[i].vx - ego_log[i-1].vx) / Ts_);
            acc_resp[1].push_back((ego_log[i].vy - ego_log[i-1].vy) / Ts_);
            acc_resp[2].push_back((ego_log[i].vz - ego_log[i-1].vz) / Ts_);
            time_resp.push_back(i * Ts_);
        }

        const char* comp_name[3] = {"ax", "ay", "az"};
        const bool has_pid = (pid_accel_log.size() == mpc_accel_log.size());
        for (int c = 0; c < 3; ++c) {
            plt::subplot(3, 1, c + 1);

            std::vector<double> mpc_a, pid_a;
            mpc_a.reserve(n); pid_a.reserve(n);
            for (size_t i = 0; i < n; ++i) {
                mpc_a.push_back(mpc_accel_log[i][c]);
                pid_a.push_back(has_pid ? pid_accel_log[i][c] : 0.0);
            }

            // 图例只在第一行标注一次
            std::map<std::string, std::string> kw_mpc = {{"color", "blue"}, {"linewidth", "2"}};
            std::map<std::string, std::string> kw_pid = {{"color", "darkorange"}, {"linestyle", "--"}, {"linewidth", "2"}};
            std::map<std::string, std::string> kw_rsp = {{"color", "green"}, {"linestyle", "-."}, {"linewidth", "1.5"}};
            if (c == 0) {
                kw_mpc["label"] = "MPC Accel Cmd";
                kw_pid["label"] = "PID Smoothed Cmd";
                kw_rsp["label"] = "Actual Accel (v diff)";
                plt::title("Window 9: Accel Chain: MPC Cmd vs PID Smoothed Cmd vs Actual Response");
            }
            plt::plot(time_a, mpc_a, kw_mpc);
            plt::plot(time_a, pid_a, kw_pid);
            plt::plot(time_resp, acc_resp[c], kw_rsp);

            plt::ylabel(std::string(comp_name[c]) + " [m/s^2]");
            plt::grid(true);
            if (c == 0) plt::legend();
        }
        plt::xlabel("Time [s]"); // 作用于最后一行子图
    }

    std::cout << "[Geffen Visualizer] Rendering 9 independent windows. Close ALL windows to exit." << std::endl;
    plt::show();
}

std::array<double,4> estimateMotorPWM(const Input& a_cmd)
{
    // X 型四旋翼简化推力分配模型（PX4 iris 输出序号：M1前右 M2后左 M3前左 M4后右）
    // 假设：
    //  1) MAVROS 加速度指令为机动加速度，比力 f = a_cmd + g·ez（ENU, z 向上）
    //  2) 机体 z 轴对准 f 方向 => 得到滚转/俯仰指令；偏航不参与分配
    //  3) 总推力以悬停推力(mg)为单位，满油门推重比按 2.5 计
    //  4) 滚转>0(右倾) => 右侧电机减速；俯仰>0(抬头) => 前侧电机加速
    const double g = 9.81;
    Eigen::Vector3d f(a_cmd.ax, a_cmd.ay, a_cmd.az + g);
    double fmag = f.norm();
    if (fmag < 1e-6) { f << 0.0, 0.0, g; fmag = g; }

    const double roll  = std::atan2(f.y(), f.z());
    const double pitch = -std::atan2(f.x(), std::sqrt(f.y() * f.y() + f.z() * f.z()));

    const double thrust = fmag / g; // 相对悬停推力（悬停 = 1）
    const double t_max  = 2.5;      // 满油门推重比假设
    const double k_mix  = 0.5;      // 滚转/俯仰分配增益

    double delta[4];
    delta[0] = -k_mix * roll + k_mix * pitch; // M1 前右
    delta[1] = +k_mix * roll - k_mix * pitch; // M2 后左
    delta[2] = +k_mix * roll + k_mix * pitch; // M3 前左
    delta[3] = -k_mix * roll - k_mix * pitch; // M4 后右

    std::array<double,4> pwm;
    for (int i = 0; i < 4; ++i) {
        double throttle = (thrust + delta[i]) / t_max; // 归一化油门 0~1
        throttle = std::max(0.0, std::min(1.0, throttle));
        pwm[i] = 1000.0 + 1000.0 * throttle;           // 映射到 PWM µs
    }
    return pwm;
}

} // namespace mpcc
