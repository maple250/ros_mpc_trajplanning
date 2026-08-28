#include "AccelToAttitudeController.h"
#include "Frames/frames.h"
#include <cmath>

namespace mpcc{

AccelToAttitudeController::AccelToAttitudeController(double gravity_val, double max_tilt_deg)
    : gravity_(gravity_val), max_tilt_(drone_dynamic::degToRad(max_tilt_deg)) {}

AttThrustSp AccelToAttitudeController::update(const Eigen::Vector3d &a_des_enu, double yaw_sp) const {
    //=============== 1. 总比力需求（ENU）：机动加速度 + 重力补偿 ===============
    Eigen::Vector3d f_des = a_des_enu + Eigen::Vector3d(0.0, 0.0, gravity_);

    //=============== 2. 倾角限幅：水平分量等比缩放（垂向分量保持） ===============
    //    倾角 = atan(|f_hor| / f_z)，超限时 |f_hor| 缩到 f_z·tan(max_tilt)
    const double f_hor = f_des.head<2>().norm();
    const double f_z   = f_des.z();
    if (f_z > 1e-6 && f_hor > f_z * std::tan(max_tilt_)) {
        f_des.head<2>() *= (f_z * std::tan(max_tilt_)) / f_hor;
    }

    const double f_norm = f_des.norm();
    AttThrustSp out;
    out.thrustSp = f_norm;

    //兜底：f_des 垂直向下（指令严重超出电机权限）或退化时姿态无解，保持水平姿态
    if (f_norm < 1e-6 || f_z <= 1e-6) {
        out.attSp = drone_dynamic::EulerAngle{};
        return out;
    }

    //=============== 3. 期望机体轴（world 系表达，body->world） ===============
    const Eigen::Vector3d z_b = f_des / f_norm;
    //期望航向单位矢量（ENU）
    Eigen::Vector3d x_c(std::cos(yaw_sp), std::sin(yaw_sp), 0.0);
    //y_b = z_b × x_c：z_b 水平（90°倾角，已被限幅挡住）时退化，回退最小航向兜底
    Eigen::Vector3d y_b = z_b.cross(x_c);
    if (y_b.norm() < 1e-3) {
        x_c = Eigen::Vector3d(1.0, 0.0, 0.0);
        y_b = z_b.cross(x_c);
        if (y_b.norm() < 1e-9) {
            out.attSp = drone_dynamic::EulerAngle{};
            return out;
        }
    }
    y_b.normalize();
    Eigen::Vector3d x_b = y_b.cross(z_b);
    x_b.normalize();

    //=============== 4. 构造旋转矩阵（列 = 机体轴 world 表达）并转 FLU 欧拉 ===============
    double R[3][3];
    R[0][0] = x_b.x(); R[0][1] = y_b.x(); R[0][2] = z_b.x();
    R[1][0] = x_b.y(); R[1][1] = y_b.y(); R[1][2] = z_b.y();
    R[2][0] = x_b.z(); R[2][1] = y_b.z(); R[2][2] = z_b.z();
    const drone_dynamic::Quaternion q = drone_dynamic::rotationMatrixToQuat(R);
    out.attSp = drone_dynamic::quatToEuler(q);
    return out;
}

}
