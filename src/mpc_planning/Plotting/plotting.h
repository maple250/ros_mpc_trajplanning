#ifndef MPCC_PLOTTING_H
#define MPCC_PLOTTING_H

#include "config.h"
#include "types.h"
#include <matplotlibcpp.h>
#include <vector>
#include "MPC/mpc.h"

namespace plt = matplotlibcpp;

namespace mpcc {
class Plotting {
public:
    Plotting(double Ts, const PathToJson& path);
    
    // 战术拦截专用绘图函数：同时绘制我方实际轨迹、敌方实际轨迹、以及特定时刻的MPC预测段
    void plotIntercept(const std::vector<State>& ego_log,
                       const std::vector<TargetState>& target_log,
                       const std::vector<TrackPackage>& plan_log) const;

};
}

#endif //MPCC_PLOTTING_H