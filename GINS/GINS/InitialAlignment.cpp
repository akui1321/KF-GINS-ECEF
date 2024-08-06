#include "InitialAlignment.h"
#include "utils.h"

// Matrix initialization
InitialAlignment::InitialAlignment(const std::vector<std::vector<double>>& alignment, const double& startTime, const double& endTime, const GINSConfig& config_) {
  data = calculateAverages(alignment, startTime, endTime);
  
  omega_b = Eigen::MatrixXd(3, 1);
  omega_n = Eigen::MatrixXd(3, 1);
  g_b = Eigen::MatrixXd(3, 1);
  g_n = Eigen::MatrixXd(3, 1);
  C = Eigen::MatrixXd(3, 3);

  for (int i = 0; i < 3; i++) {
    omega_b(i, 0) = data[i + 1];
    g_b(i, 0) = -data[i + 4];
  }

  omega_n(0, 0) = 0;
  omega_n(1, 0) = omega_e * cos(config_.init_pos[0] / 180.0 * pi);
  omega_n(2, 0) = omega_e * sin(config_.init_pos[0] / 180.0 * pi);

  g = BLH2g(config_.init_pos[0] / 180.0 * pi, config_.init_pos[1] / 180.0 * pi, config_.init_pos[2] / 180.0 * pi);
  g_n = Eigen::MatrixXd(3, 1);
  g_n.fill(0);
  g_n(2, 0) = -g;

  C.fill(0);
}

// Calculate the average of the static data
std::vector<double> InitialAlignment::calculateAverages(const std::vector<std::vector<double>>& alignment, const double& startTime, const double& endTime) {
  int numValues = alignment.empty() ? 0 : alignment[0].size();
  std::vector<double> averages(numValues, 0.0);
  int numEntries = 0;

  int bias = 0;

  for (const auto& entry : alignment) {
    if (entry.size() > 1 && entry[0] >= startTime && entry[0] <= endTime) {
      if (entry[1] > (0.002 * 200.0)) {
        bias++;
        continue;
      }

      for (int i = 0; i < numValues; ++i) {
        averages[i] += entry[i];
      }
      numEntries++;
    }
  }

  if (numEntries > 0) {
    for (double& average : averages) {
      average /= numEntries;
    }
  }

  return averages;
}

// Solve for the attitude matrix
void InitialAlignment::calculateMatrix() {
  Eigen::MatrixXd v_g = g_n / Magnitude(g_n);
  Eigen::MatrixXd v_w = CrossProduct(g_n, omega_n) / Magnitude(CrossProduct(g_n, omega_n));
  Eigen::MatrixXd v_gw = CrossProduct(CrossProduct(g_n, omega_n), g_n) / Magnitude(CrossProduct(CrossProduct(g_n, omega_n), g_n));

  Eigen::MatrixXd w_g = g_b / Magnitude(g_b);
  Eigen::MatrixXd w_w = CrossProduct(g_b, omega_b) / Magnitude(CrossProduct(g_b, omega_b));
  Eigen::MatrixXd w_gw = CrossProduct(CrossProduct(g_b, omega_b), g_b) / Magnitude(CrossProduct(CrossProduct(g_b, omega_b), g_b));

  Eigen::MatrixXd V(3, 3), W(3, 3);
  V << v_g, v_w, v_gw;
  W << w_g, w_w, w_gw;
  C = V * W.transpose();

  Mat2Euler(C, pitch, roll, yaw);
}

double InitialAlignment::getRoll() const {
  return roll;
}

double InitialAlignment::getPitch() const {
  return pitch;
}

double InitialAlignment::getYaw() const {
  return yaw;
}
