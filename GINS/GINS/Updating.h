#ifndef UPDATING_H
#define UPDATING_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "utils.h"

#define omega_e 7.292115e-5
#define a 6378137
#define e 0.08181919104
#define g 9.7936174
#define pi 3.1415926535897932384626433832795028841971

/**
 * @class Updating
 * A class responsible for updating the states of the INS (Inertial Navigation System)
 * using measurements from an epoch to the next.
 */
class Updating {
public:
  Updating();
  void Init(std::vector<double> epoch1, std::vector<double> epoch2, Eigen::MatrixXd v_k1, Eigen::MatrixXd pos_k1, Eigen::MatrixXd att_k1);
  void calculate(std::vector<double> epoch1_, std::vector<double> epoch2_);
  Eigen::MatrixXd att_k2;
  Eigen::MatrixXd pos_k2;
  Eigen::MatrixXd v_k2;
  Eigen::MatrixXd q_b_e_ins;

private:
  std::vector<double> epoch1;
  std::vector<double> epoch2;
  Eigen::MatrixXd delta_theta_k1;
  Eigen::MatrixXd delta_v_k1;
  Eigen::MatrixXd delta_theta_k2;
  Eigen::MatrixXd delta_v_k2;
  Eigen::MatrixXd delta_v_b_k;
  Eigen::MatrixXd delta_v_fk;
  Eigen::MatrixXd delta_v_cor;
  Eigen::MatrixXd v_k1;
  Eigen::MatrixXd pos_k1;
  Eigen::MatrixXd att_k1;
  GINSConfig config;

  Eigen::MatrixXd omega_ie_e;

  Eigen::MatrixXd phi_k;
  Eigen::MatrixXd xi_k;

  double delta_t = 0.005;
};

#endif