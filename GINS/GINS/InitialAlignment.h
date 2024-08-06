#ifndef INITIAL_ALIGNMENT_H
#define INITIAL_ALIGNMENT_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "utils.h"

#define pi 3.1415926535897932384626433832795028841971
#define omega_e 7.292115e-5

/**
 * @class InitialAlignment
 * A class for performing the initial alignment of an IMU.
 */
class InitialAlignment {
public:
  InitialAlignment(const std::vector<std::vector<double>>& alignment, const double& startTime, const double& endTime, const GINSConfig& config_);
  void calculateMatrix();
  double getRoll() const;
  double getPitch() const;
  double getYaw() const;

private:
  Eigen::MatrixXd g_b;
  Eigen::MatrixXd omega_b;
  Eigen::MatrixXd C;
  double yaw;
  double pitch;
  double roll;
  double g;
  std::vector<double> data;

  Eigen::MatrixXd g_n;
  Eigen::MatrixXd omega_n;

  std::vector<double> calculateAverages(const std::vector<std::vector<double>>& alignment, const double& startTime, const double& endTime);
};

#endif
