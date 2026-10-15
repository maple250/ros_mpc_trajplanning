#include "INDIAttitudeController.h"
#include "Frames/frames.h"
#include <algorithm>
#include <cmath>

namespace mpcc{

namespace {
//一阶低通（前向欧拉，同 indi 工程 low_pass_filter：y += 2π·fc·(x−y)·dt）
double lowPass(double input, double fc_hz, double output, double dt) {
    return output + 2.0 * M_PI * fc_hz * (input - output) * dt;
}
//推力归一化下限 [m/s²]，防比力模值接近零（失重）时增量除零
constexpr double kMinThrustNorm = 1.0;
//雅可比退化判据：|det M| = |cosθ·cos²φ| 过小（接近 ±90° 倾角）时放弃横向增量，保持滤波基准
constexpr double kMinDet = 1e-3;
//推力换算的 cosφ·cosθ 下限，防倾角过大时推力增益爆炸
constexpr double kMinTiltCos = 0.1;
}

INDIAttitudeController::INDIAttitudeController(double gravity_val, const INDIParam &param)
    : gravity_(gravity_val), param_(param), initialized_(false),
      roll_base_(0.0), pitch_base_(0.0), thrust_base_(gravity_val), thrust_int_(0.0) {}

AttThrustSp INDIAttitudeController::update(const Eigen::Vector3d &a_des_enu, const Eigen::Vector3d &a_meas_enu,
                                           const drone_dynamic::EulerAngle &att_meas, double yaw_sp, double dt) {
    const double phi = att_meas.roll, theta = att_meas.pitch, psi = att_meas.yaw;
    //首拍：滤波基准以测量姿态初始化，避免从零角度起步的瞬态
    if (!initialized_) {
        roll_base_  = phi;
        pitch_base_ = theta;
        initialized_ = true;
    }

    //=============== 1. 运动加速度误差（比力去重力；水平分量与比力相同，z 分量用于推力） ===============
    const Eigen::Vector3d a_meas_motion = a_meas_enu - Eigen::Vector3d(0.0, 0.0, gravity_);
    const Eigen::Vector3d a_err = a_des_enu - a_meas_motion;

    //=============== 2. 横向增量：Δ(φ,θ) = M⁻¹·Δa_xy / f（M 由测量姿态解析求得，同 indi get_INDI_Matrix） ===============
    // f 取测量比力模值（即当前推力）而非常量 g：J = ∂a_xy/∂(φ,θ) 正比于推力，指令大机动时推力可达 2~4g，
    // 用 g 归一化会使增量按推力比例失准（indi 原工程 at=g 仅在悬停附近成立）
    const double f_meas = std::max(a_meas_enu.norm(), kMinThrustNorm);
    const double cph = std::cos(phi),   sph = std::sin(phi);
    const double cth = std::cos(theta), sth = std::sin(theta);
    const double cps = std::cos(psi),   sps = std::sin(psi);
    Eigen::Matrix2d M;
    M << sps * cph - cps * sth * sph, cps * cth * cph,
        -cps * cph - sps * sth * sph, sps * cth * cph;
    Eigen::Vector2d d_rp = Eigen::Vector2d::Zero();
    if (std::fabs(M.determinant()) > kMinDet) {
        d_rp = M.inverse() * (a_err.head<2>() / f_meas);
    }

    //=============== 3. 推力增量：T = T_base + Kp·Δa_z/(cosφcosθ) + Ki·∫Δa_z（同 indi calculate_Throttle） ===============
    const double dAz = a_err.z();
    const double cos_tilt = std::max(cph * cth, kMinTiltCos);
    const double thrust_cmd = thrust_base_ + param_.kp_thrust * dAz / cos_tilt + param_.ki_thrust * thrust_int_;
    thrust_int_ = std::min(std::max(thrust_int_ + dAz * dt, -param_.thrust_int_lim), param_.thrust_int_lim);

    //=============== 4. 输出：姿态 = 滤波测量基准 + 增量（倾角限幅），航向取偏航设定 ===============
    const double max_tilt = drone_dynamic::degToRad(param_.max_tilt_deg);
    AttThrustSp out;
    out.attSp.roll  = std::min(std::max(roll_base_  + d_rp.x(), -max_tilt), max_tilt);
    out.attSp.pitch = std::min(std::max(pitch_base_ + d_rp.y(), -max_tilt), max_tilt);
    out.attSp.yaw   = yaw_sp;
    out.thrustSp    = std::max(thrust_cmd, 0.0);

    //=============== 5. 滤波基准更新（供下一拍增量使用） ===============
    roll_base_   = lowPass(phi,       param_.att_lpf_hz,    roll_base_,   dt);
    pitch_base_  = lowPass(theta,     param_.att_lpf_hz,    pitch_base_,  dt);
    thrust_base_ = lowPass(thrust_cmd, param_.thrust_lpf_hz, thrust_base_, dt);
    return out;
}

}
