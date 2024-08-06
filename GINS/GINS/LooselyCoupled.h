#ifndef LOOSELYCOUPLED_H
#define LOOSELYCOUPLED_H

#include <vector>
#include <fstream>
#include "utils.h"

/**
 * @class LooselyCoupled
 * Entry point of the loosely coupled algorithm.
 * In this approach, the GNSS and INS systems operate independently but exchange information at certain intervals.
 */
class LooselyCoupled {
public:
  LooselyCoupled(const std::vector<std::vector<double>>& imu_data,
    const std::vector<std::vector<double>>& gnss_data,
    const std::vector<std::vector<double>>& pos_results,
    const GINSConfig& config);

  bool process();

private:
  std::vector<std::vector<double>> imu_data_;
  std::vector<std::vector<double>> gnss_data_;
  std::vector<std::vector<double>> pos_results_;
  GINSConfig config_;

  std::vector<double> interpolate(std::vector<double> last_imu_data, std::vector<double> imu_data, double time);

  void ProgressBar(const int& max_num, const int& i);
};

#endif