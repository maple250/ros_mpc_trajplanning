#include "plotting.h"
#include <cmath>

namespace mpcc {

Plotting::Plotting(double Ts, const PathToJson& path)
{
}

void Plotting::plotIntercept(const std::vector<State>& ego_log,
                             const std::vector<TargetState>& target_log,
                             const std::vector<TrackPackage>& plan_log) const
{
    std::vector<double> ego_x, ego_y, ego_z, ego_v;
    std::vector<double> target_x, target_y, target_z;
    std::vector<double> dist, time;

    double Ts_val = 0.1; 
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
        t += Ts_val;
    }

    // ==========================================
    // 窗口 1: XY 平面视图
    // ==========================================
    plt::figure_size(600, 600);
    plt::plot(ego_x, ego_y, {{"label", "Ego Traj"}, {"color", "blue"}, {"linewidth", "2"}});
    plt::plot(target_x, target_y, {{"label", "Target Traj"}, {"color", "red"}, {"linewidth", "2"}});
    plt::plot({ego_x.front()}, {ego_y.front()}, {{"marker", "*"}, {"color", "black"}, {"markersize", "10"}});
    plt::plot({target_x.front()}, {target_y.front()}, {{"marker", "*"}, {"color", "black"}, {"markersize", "10"}});
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

    std::cout << "[Geffen Visualizer] Rendering 4 independent windows. Close ALL windows to exit." << std::endl;
    plt::show(); 
}

} // namespace mpcc