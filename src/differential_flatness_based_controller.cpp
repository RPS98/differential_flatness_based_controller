#include "differential_flatness_based_controller.hpp"

using namespace df_controller;

DF::DF() { pid_handler_ = pid_controller::PIDController3D(); }

DF::~DF() {}

void DF::setGains(Vector3d _kp, Vector3d _ki, Vector3d _kd) {
  pid_handler_.setGains(_kp, _ki, _kd);
}

void DF::setGainKpX(double _kp) { pid_handler_.setGainKpX(_kp); }

void DF::setGainKpY(double _kp) { pid_handler_.setGainKpY(_kp); }

void DF::setGainKpZ(double _kp) { pid_handler_.setGainKpZ(_kp); }

void DF::setGainKiX(double _ki) { pid_handler_.setGainKiX(_ki); }

void DF::setGainKiY(double _ki) { pid_handler_.setGainKiY(_ki); }

void DF::setGainKiZ(double _ki) { pid_handler_.setGainKiZ(_ki); }

void DF::setGainKdX(double _kd) { pid_handler_.setGainKdX(_kd); }

void DF::setGainKdY(double _kd) { pid_handler_.setGainKdY(_kd); }

void DF::setGainKdZ(double _kd) { pid_handler_.setGainKdZ(_kd); }

void DF::setAntiWindup(Vector3d _anti_windup) { pid_handler_.setAntiWindup(_anti_windup); }

void DF::setAntiWindup(double _anti_windup) { pid_handler_.setAntiWindup(_anti_windup); }

void DF::setAlpha(Vector3d _alpha) { pid_handler_.setAlpha(_alpha); }

void DF::setAlpha(double _alpha) { pid_handler_.setAlpha(_alpha); }

void DF::setResetIntegralSaturationFlag(bool _reset_integral_flag) {
  pid_handler_.setResetIntegralSaturationFlag(_reset_integral_flag);
}

void DF::setMass(double _mass) { mass_ = _mass; }

void DF::setGravitationalAcc(Vector3d _gravitational_acc) {
  gravitational_accel_ = _gravitational_acc;
}

void DF::setGravitationalAcc(double _gravitational_acc) {
  gravitational_accel_ = Vector3d(0.0, 0.0, _gravitational_acc);
}

void DF::setGainsAngular(Vector3d _kp) { Kp_ang_mat_ = _kp.asDiagonal(); }

void DF::setGainKpRollAngular(double _kp) { Kp_ang_mat_(0, 0) = _kp; }

void DF::setGainKpPitchAngular(double _kp) { Kp_ang_mat_(1, 1) = _kp; }

void DF::setGainKpYawAngular(double _kp) { Kp_ang_mat_(2, 2) = _kp; }

Vector3d DF::getForce(const double &_dt,
                      const Vector3d &_pos_state,
                      const Vector3d &_vel_state,
                      const Vector3d &_pos_reference,
                      const Vector3d &_vel_reference,
                      const Vector3d &_acc_reference) {
  // Compute the error force contribution
  Vector3d force_error =
      pid_handler_.computeControl(_dt, _pos_state, _pos_reference, _vel_state, _vel_reference);

  // Compute acceleration reference contribution
  Vector3d force_acceleration = mass_ * _acc_reference;

  // Compute gravity compensation
  Vector3d force_gravity = mass_ * gravitational_accel_;

  // Return desired force with the gravity compensation
  Vector3d desired_force = force_error + force_acceleration + force_gravity;
  return desired_force;
}

Acro_command DF::computeTrajectoryControl(const double &_dt,
                                          const Vector3d &_pos_state,
                                          const Vector3d &_vel_state,
                                          const tf2::Quaternion &_attitude_state,
                                          const Vector3d &_pos_reference,
                                          const Vector3d &_vel_reference,
                                          const Vector3d &_acc_reference,
                                          const double &_yaw_angle_reference) {
  Vector3d desired_force =
      getForce(_dt, _pos_state, _vel_state, _pos_reference, _vel_reference, _acc_reference);

  // Compute the desired attitude
  tf2::Matrix3x3 rot_matrix_tf2(_attitude_state);

  Eigen::Matrix3d rot_matrix;
  rot_matrix << rot_matrix_tf2[0][0], rot_matrix_tf2[0][1], rot_matrix_tf2[0][2],
      rot_matrix_tf2[1][0], rot_matrix_tf2[1][1], rot_matrix_tf2[1][2], rot_matrix_tf2[2][0],
      rot_matrix_tf2[2][1], rot_matrix_tf2[2][2];

  Vector3d xc_des(cos(_yaw_angle_reference), sin(_yaw_angle_reference), 0);

  Vector3d zb_des = desired_force.normalized();
  Vector3d yb_des = zb_des.cross(xc_des).normalized();
  Vector3d xb_des = yb_des.cross(zb_des).normalized();

  // Compute the rotation matrix desidered
  Eigen::Matrix3d R_des;
  R_des.col(0) = xb_des;
  R_des.col(1) = yb_des;
  R_des.col(2) = zb_des;

  // Compute the rotation matrix error
  Eigen::Matrix3d Mat_e_rot = (R_des.transpose() * rot_matrix - rot_matrix.transpose() * R_des);

  Vector3d V_e_rot(Mat_e_rot(2, 1), Mat_e_rot(0, 2), Mat_e_rot(1, 0));
  Vector3d E_rot = (1.0f / 2.0f) * V_e_rot;

  Acro_command acro_command;
  acro_command.thrust = (float)desired_force.dot(rot_matrix.col(2).normalized());
  acro_command.PQR    = -Kp_ang_mat_ * E_rot;

  return acro_command;
}

void DF::resetController() { pid_handler_.resetController(); }
