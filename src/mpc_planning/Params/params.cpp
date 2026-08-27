//类特点：成员变量粗分类、存储参数，成员函数负责从json文件读取参数
#include "params.h"
#include <cmath>
namespace mpcc{
CostParam::CostParam(){
    std::cout << "Default initialization of cost" << std::endl;
}

CostParam::CostParam(std::string file){
    /////////////////////////////////////////////////////
    // Loading Cost Parameters //////////////////////////
    /////////////////////////////////////////////////////
    std::ifstream iCost(file);
    json jsonCost;
    iCost >> jsonCost;
    // 1. 法向位置误差收敛项
    q_c          = jsonCost["q_c"];
    // 4. 速度跟踪二次项（固定权重，随距离调制）
    q_v          = jsonCost["q_v"];
    // 5. 加速度指令惩罚项
    q_a          = jsonCost["q_a"];
    // 2. 速度大小奖励项（高斯变权重）
    q_vmag       = jsonCost["q_vmag"];
    sigma_v      = jsonCost["sigma_v"];
    // 3. APN 速度增量项（tanh 变权重）
    q_dv         = jsonCost["q_dv"];
    N_apn        = jsonCost["N_apn"];
    rho_dv       = jsonCost["rho_dv"];
    k_dv         = jsonCost["k_dv"];
    // 4. 速度跟踪权重 tanh 调制参数
    rho_v        = jsonCost["rho_v"];
    k_v          = jsonCost["k_v"];
}

BoundsParam::BoundsParam() {
    std::cout << "Default initialization of bounds" << std::endl;
}

BoundsParam::BoundsParam(std::string file) {
    /////////////////////////////////////////////////////
    // Loading Bounds Parameters //////////////////////////
    /////////////////////////////////////////////////////
    std::ifstream iBounds(file);
    json jsonBounds;
    iBounds >> jsonBounds;
    //命名规定：在json文件变量命名中，本身没有“_”  !!!
    lower_state_bounds.px_l = jsonBounds["px_l"];
    lower_state_bounds.py_l = jsonBounds["py_l"];
    lower_state_bounds.pz_l = jsonBounds["pz_l"];
    lower_state_bounds.vx_l = jsonBounds["vx_l"];
    lower_state_bounds.vy_l = jsonBounds["vy_l"];
    lower_state_bounds.vz_l = jsonBounds["vz_l"];

    upper_state_bounds.px_u = jsonBounds["px_u"];
    upper_state_bounds.py_u = jsonBounds["py_u"];
    upper_state_bounds.pz_u = jsonBounds["pz_u"];
    upper_state_bounds.vx_u = jsonBounds["vx_u"];
    upper_state_bounds.vy_u = jsonBounds["vy_u"];
    upper_state_bounds.vz_u = jsonBounds["vz_u"];

    lower_input_bounds.ax_l = jsonBounds["ax_l"];
    lower_input_bounds.ay_l = jsonBounds["ay_l"];
    lower_input_bounds.az_l = jsonBounds["az_l"];

    upper_input_bounds.ax_u = jsonBounds["ax_u"];
    upper_input_bounds.ay_u = jsonBounds["ay_u"];
    upper_input_bounds.az_u = jsonBounds["az_u"];
}

NormalizationParam::NormalizationParam(){
    std::cout << "Default initialization of normalization" << std::endl;
}

NormalizationParam::NormalizationParam(std::string file)
{
    /////////////////////////////////////////////////////
    // Loading Normalization Parameters /////////////////
    /////////////////////////////////////////////////////
    std::ifstream iNorm(file);
    json jsonNorm;
    iNorm >> jsonNorm;
    T_x.setIdentity();
    T_x(si_index.px,si_index.px) = jsonNorm["px"];
    T_x(si_index.py,si_index.py) = jsonNorm["py"];
    T_x(si_index.pz,si_index.pz) = jsonNorm["pz"];
    T_x(si_index.vx,si_index.vx) = jsonNorm["vx"];
    T_x(si_index.vy,si_index.vy) = jsonNorm["vy"];
    T_x(si_index.vz,si_index.vz) = jsonNorm["vz"];
    T_x_inv.setIdentity();
    for(int i = 0;i<NX;i++)
    {
        T_x_inv(i,i) = 1.0/T_x(i,i);
    }
    T_u.setIdentity();
    T_u(si_index.ax,si_index.ax) = jsonNorm["ax"];
    T_u(si_index.ay,si_index.ay) = jsonNorm["ay"];
    T_u(si_index.az,si_index.az) = jsonNorm["az"];
    T_u_inv.setIdentity();
    for(int i = 0;i<NU;i++)
    {
        T_u_inv(i,i) = 1.0/T_u(i,i);
    }
    T_s.setIdentity();
    T_s_inv.setIdentity();
}

InitialParam::InitialParam(){
    std::cout << "Default initialization of initial State" << std::endl;
}
InitialParam::InitialParam(std::string file){
    std::ifstream iNorm(file);
    json jsonNorm;
    iNorm >> jsonNorm;
    Pos_target_init.x() = jsonNorm["pxt_init"];
    Pos_target_init.y() = jsonNorm["pyt_init"];
    Pos_target_init.z() = jsonNorm["pzt_init"];
    //目标运动特性：水平/垂直速度大小 + 水平速度方位角 theta (deg)
    Target_vel_h    = jsonNorm["vht_init"];
    Target_vel_v    = jsonNorm["vzt_init"];
    Target_theta    = jsonNorm["theta_init"];
    Target_movetype = jsonNorm["movetype_init"];
    Target_circle_R = jsonNorm["Rt_init"];
    //换算惯性系初始速度分量：vx = Vh*cos(theta)，vy = Vh*sin(theta)，vz 直接取垂直速度
    double theta_rad = Target_theta * M_PI / 180.0;
    Vel_target_init.x() = Target_vel_h * std::cos(theta_rad);
    Vel_target_init.y() = Target_vel_h * std::sin(theta_rad);
    Vel_target_init.z() = Target_vel_v;

    Pos_self_init.x() = jsonNorm["pxs_init"];
    Pos_self_init.y() = jsonNorm["pys_init"];
    Pos_self_init.z() = jsonNorm["pzs_init"];
    Vel_self_init.x() = jsonNorm["vxs_init"];
    Vel_self_init.y() = jsonNorm["vys_init"];
    Vel_self_init.z() = jsonNorm["vzs_init"];
}
}
