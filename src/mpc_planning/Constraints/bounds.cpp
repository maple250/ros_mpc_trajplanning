//类特点：成员变量精细分类、存储参数（结构体形式），成员函数直接输出某一类参数
#include "bounds.h"
namespace mpcc{
Bounds::Bounds()
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}
//初始化私有成员变量为BoundsParam类中的lower_state_bounds等结构对象的各个边界值
Bounds::Bounds(BoundsParam bounds_param) 
{
    l_bounds_x_(0) = bounds_param.lower_state_bounds.px_l;
    l_bounds_x_(1) = bounds_param.lower_state_bounds.py_l;
    l_bounds_x_(2) = bounds_param.lower_state_bounds.pz_l;
    l_bounds_x_(3) = bounds_param.lower_state_bounds.vx_l;
    l_bounds_x_(4) = bounds_param.lower_state_bounds.vy_l;
    l_bounds_x_(5) = bounds_param.lower_state_bounds.vz_l;

    u_bounds_x_(0) = bounds_param.upper_state_bounds.px_u;
    u_bounds_x_(1) = bounds_param.upper_state_bounds.py_u;
    u_bounds_x_(2) = bounds_param.upper_state_bounds.pz_u;
    u_bounds_x_(3) = bounds_param.upper_state_bounds.vx_u;
    u_bounds_x_(4) = bounds_param.upper_state_bounds.vy_u;
    u_bounds_x_(5) = bounds_param.upper_state_bounds.vz_u;

    l_bounds_u_(0) = bounds_param.lower_input_bounds.ax_l;
    l_bounds_u_(1) = bounds_param.lower_input_bounds.ay_l;
    l_bounds_u_(2) = bounds_param.lower_input_bounds.az_l;

    u_bounds_u_(0) = bounds_param.upper_input_bounds.ax_u;
    u_bounds_u_(1) = bounds_param.upper_input_bounds.ay_u;
    u_bounds_u_(2) = bounds_param.upper_input_bounds.az_u;

    l_bounds_s_ = Bounds_s::Zero();//软约束边界
    u_bounds_s_ = Bounds_s::Zero();

    std::cout << "bounds initialized" << std::endl;
}
//输出Eigen矩阵类型（自定义Bounds_x类型）的私有成员变量值（边界值）
Bounds_x Bounds::getBoundsLX() const
{
    return  l_bounds_x_;
}
//输出Eigen矩阵类型（自定义Bounds_x类型）的私有成员变量值（边界值）
Bounds_x Bounds::getBoundsUX() const
{
    return  u_bounds_x_;
}
//输出Eigen矩阵类型（自定义Bounds_u类型）的私有成员变量值（边界值）
Bounds_u Bounds::getBoundsLU() const
{
    return  l_bounds_u_;
}
//输出Eigen矩阵类型（自定义Bounds_u类型）的私有成员变量值（边界值）
Bounds_u Bounds::getBoundsUU() const
{
    return  u_bounds_u_;
}
//输出Eigen矩阵类型（自定义Bounds_x类型）的私有成员变量值（边界值）
Bounds_s Bounds::getBoundsLS() const
{
    return  l_bounds_s_;
}
//输出Eigen矩阵类型（自定义Bounds_x类型）的私有成员变量值（边界值）
Bounds_s Bounds::getBoundsUS() const{
    return  u_bounds_s_;
}
}