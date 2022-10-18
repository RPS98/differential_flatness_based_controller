#ifndef DF_CONTROLLER_3D_HPP_
#define DF_CONTROLLER_3D_HPP_

#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <pid_controller/PID_3D.hpp>

namespace df_controller {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

struct Acro_command {
  Vector3d PQR;
  double thrust;
};

class DF {
public:
  DF();
  ~DF();

  void setGains(Vector3d _kp, Vector3d _ki, Vector3d _kd);
  void setGainKpX(double _kp);
  void setGainKpY(double _kp);
  void setGainKpZ(double _kp);
  void setGainKiX(double _ki);
  void setGainKiY(double _ki);
  void setGainKiZ(double _ki);
  void setGainKdX(double _kd);
  void setGainKdY(double _kd);
  void setGainKdZ(double _kd);
  void setAntiWindup(Vector3d _anti_windup);
  void setAntiWindup(double _anti_windup);
  void setAlpha(Vector3d _alpha);
  void setAlpha(double _alpha);
  void setResetIntegralSaturationFlag(bool _reset_integral_flag);

  void setMass(double _mass);
  void setGravitationalAcc(Vector3d _gravitational_acc);
  void setGravitationalAcc(double _gravitational_acc);
  void setGainsAngular(Vector3d _kp);
  void setGainKpRollAngular(double _kp);
  void setGainKpPitchAngular(double _kp);
  void setGainKpYawAngular(double _kp);

  Vector3d getForce(const double &_dt,
                    const Vector3d &_pos_state,
                    const Vector3d &_vel_state,
                    const Vector3d &_pos_reference,
                    const Vector3d &_vel_reference,
                    const Vector3d &_acc_reference);

  Acro_command computeTrajectoryControl(const double &_dt,
                                        const Vector3d &_pos_state,
                                        const Vector3d &_vel_state,
                                        const tf2::Quaternion &_attitude_state,
                                        const Vector3d &_pos_reference,
                                        const Vector3d &_vel_reference,
                                        const Vector3d &_acc_reference,
                                        const double &_yaw_angle_reference);

  void resetController();

private:
  pid_controller::PIDController3D pid_handler_;

  double mass_;
  Vector3d gravitational_accel_ = Vector3d(0, 0, 9.81);
  Eigen::Matrix3d Kp_ang_mat_   = Eigen::Matrix3d::Zero();
};
}  // namespace df_controller

#endif  // DF_CONTROLLER_3D_HPP_
