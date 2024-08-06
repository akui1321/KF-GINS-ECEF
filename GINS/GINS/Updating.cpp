#include "Updating.h"
#include "utils.h"
#include <iomanip>

Updating::Updating() {}

// Data initialization
void Updating::Init(std::vector<double> epoch1_, std::vector<double> epoch2_, Eigen::MatrixXd v_k1_, Eigen::MatrixXd pos_k1_, Eigen::MatrixXd att_k1_) {
  epoch1 = epoch1_;
  epoch2 = epoch2_;
  v_k1 = v_k1_;
  pos_k1 = pos_k1_;
  att_k1 = att_k1_; 
  delta_theta_k1 = Eigen::MatrixXd(3, 1);
  delta_v_k1 = Eigen::MatrixXd(3, 1);
  delta_theta_k2 = Eigen::MatrixXd(3, 1);
  delta_v_k2 = Eigen::MatrixXd(3, 1);
  omega_ie_e = Eigen::MatrixXd(3, 1);
  delta_v_fk = Eigen::MatrixXd(3, 1);
  q_b_e_ins = Eigen::MatrixXd(3, 1);
  delta_t = epoch2[0] - epoch1[0];

  for (int i = 0; i < 3; ++i) {
    delta_theta_k1(i, 0) = epoch1[i + 1];
    delta_v_k1(i, 0) = epoch1[i + 4];
    delta_theta_k2(i, 0) = epoch2[i + 1];
    delta_v_k2(i, 0) = epoch2[i + 4];
  }

  pos_k2 = Eigen::MatrixXd(3, 1);
  v_k2 = Eigen::MatrixXd(3, 1);
  att_k2 = Eigen::MatrixXd(3, 1);

  pos_k2 = pos_k1_;
  v_k2 = v_k1_;
  att_k2 = att_k1_;

  omega_ie_e(0, 0) = 0;
  omega_ie_e(1, 0) = 0;
  omega_ie_e(2, 0) = omega_e;

  Eigen::MatrixXd q = Euler2Mat(att_k1(1, 0), att_k1(2, 0), att_k1(0, 0));

  Eigen::MatrixXd BLH = XYZtoBLH(pos_k1(0, 0), pos_k1(1, 0), pos_k1(2, 0));

  Eigen::MatrixXd Cel = E2L(BLH(0, 0), BLH(1, 0));

  q_b_e_ins = Cel.transpose() * q;
}

/**
 * @brief Updates the states of the IMU from one epoch to the next.
 *
 * This method calculates the changes in attitude, velocity, and position based on the
 * data provided for two epochs. It uses the rotation vector to update the attitude, applies
 * corrections for gravitational forces and Earth's rotation, and updates the velocity and
 * position accordingly.
 *
 * @param epoch1_ A vector containing the imu data for the first epoch.
 * @param epoch2_ A vector containing the imu data for the second epoch.
 */
void Updating::calculate(std::vector<double> epoch1_, std::vector<double> epoch2_) {
  delta_t = epoch2_[0] - epoch1_[0];

  for (int i = 0; i < 3; ++i) {
    delta_theta_k1(i, 0) = epoch1_[i + 1];
    delta_v_k1(i, 0) = epoch1_[i + 4];
    delta_theta_k2(i, 0) = epoch2_[i + 1];
    delta_v_k2(i, 0) = epoch2_[i + 4];
  }

  //Attitude updating
  Eigen::MatrixXd BLH = XYZtoBLH(pos_k1(0, 0), pos_k1(1, 0), pos_k1(2, 0));
  Eigen::MatrixXd Cel = E2L(BLH(0, 0), BLH(1, 0));

  Eigen::MatrixXd q = q_b_e_ins;

  phi_k = delta_theta_k2 * delta_t;

  Eigen::MatrixXd q_b = Vec2MatRight(phi_k);

  xi_k = omega_ie_e * delta_t;

  Eigen::MatrixXd q_e = Vec2Mat(xi_k);

  q_b_e_ins = q_e * q * q_b;

  Eigen::MatrixXd q_b_e = Cel * q_b_e_ins;

  Mat2Euler(q_b_e, att_k2(1, 0), att_k2(2, 0), att_k2(0, 0));

  // Velocity updating
  v_k1 = v_k2;

  delta_v_b_k = delta_v_k2 * delta_t;

  Eigen::MatrixXd g_l_e(3, 1);
  g_l_e.fill(0);
  double g_l = BLH2g(BLH(0, 0), BLH(1, 0), BLH(2, 0));
  g_l_e = BLH2g_e(BLH(0, 0), BLH(1, 0), g_l);

  q_b_e = Cel.transpose()* q_b_e;

  Eigen::MatrixXd omegab = q_b_e.transpose() * omega_ie_e;

  Eigen::MatrixXd delta_theta_v = delta_theta_k2 * delta_t - omegab * delta_t;

  delta_v_fk = q_b_e * (delta_v_b_k - 0.5 * CrossProduct(delta_theta_v, Eigen::Matrix3d::Identity()) * delta_v_b_k);

  delta_v_cor = (g_l_e - 2 * CrossProduct(omega_ie_e, v_k1)) * delta_t;

  v_k2 = v_k1 + delta_v_fk + delta_v_cor;

  // Position updating
  pos_k1 = pos_k2;
  pos_k2 = pos_k1 + 0.5 * (v_k1 + v_k2) * delta_t;
}
