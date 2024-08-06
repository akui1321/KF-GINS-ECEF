#include "LooselyCoupled.h"
#include "InitialAlignment.h"
#include "GinsFilter.h"
#include <iomanip>

LooselyCoupled::LooselyCoupled(const std::vector<std::vector<double>>& imu_data,
  const std::vector<std::vector<double>>& gnss_data,
  const std::vector<std::vector<double>>& pos_results,
  const GINSConfig& config)
  : imu_data_(imu_data), gnss_data_(gnss_data), pos_results_(pos_results), config_(config) {}

bool LooselyCoupled::process() {
  /**************************Preparation*****************************/
  // Filter
  GinsFilter filter;

  // Result File
  std::ofstream result_file(config_.position_result_file);
  if (!result_file.is_open()) {
    std::cerr << "Error: Could not open file " << config_.position_result_file << std::endl;
    return false;
  }

  std::ofstream diff_file(config_.position_diff_file);
  if (!diff_file.is_open()) {
    std::cerr << "Error: Could not open file " << config_.position_result_file << std::endl;
    return false;
  }

  // Initial Alignment
  double t_start, t_end;
  double init_euler[3];
  Eigen::MatrixXd ant_lever(3, 1);
  for (int i = 0; i < 3; i++) {
    init_euler[i] = config_.init_att[i];
    ant_lever(i, 0) = config_.lever[i];
  }

  if (config_.is_align) {
    t_start = imu_data_[0][0];
    t_end = config_.end_align_time;
    InitialAlignment IniAlign(imu_data_, t_start, t_end, config_);
    IniAlign.calculateMatrix();
    init_euler[0] = IniAlign.getYaw();
    init_euler[2] = IniAlign.getRoll();
    init_euler[1] = IniAlign.getPitch();
  }
  else {
    t_end = config_.end_align_time;
  }
  std::cout << std::fixed << std::setprecision(6) << "Initial Alignment Result: " <<
    "Yaw = " << Rad2Degree(init_euler[0]) << " " <<
    "Pitch = " << Rad2Degree(init_euler[1]) << " " <<
    "Roll = " << Rad2Degree(init_euler[2]) << std::endl;

  int max_epoch = imu_data_.size();

  // Filter Param
  filter.SetLeverArm(config_.lever);

  Eigen::MatrixXd pos_k1(3, 1), v_k1(3, 1), att_k1(3, 1);

  BLHtoXYZ(config_.init_pos[0], config_.init_pos[1], config_.init_pos[2], pos_k1(0, 0), pos_k1(1, 0), pos_k1(2, 0));

  Eigen::MatrixXd Cen = E2L(pos_k1(0, 0), pos_k1(1, 0));

  for (int i = 0; i < 3; i++) {
    att_k1(i, 0) = init_euler[i];
    v_k1(i, 0) = config_.init_vel[i];
  }

  v_k1 = Cen.transpose() * v_k1;

  /*******************************Solve**********************************/
  // Time Synchronize
  double i_time, g_time, r_time;
  int i = 0, k = 0, n = 0;
  Eigen::MatrixXd posBL(3, 1);

  while (fabs(imu_data_[i][0] - t_end) > 1.0E-3) {
    ++i;
    if (i == imu_data_.size() - 1) {
      std::cout << "ERROR IN END TIME! PLEASE CHECK IT." << std::endl;
      return false;
    }
  }

  while (gnss_data_[k][0] < t_end) {
    ++k;
  }

  while (pos_results_[n][0] < t_end) {
    ++n;
  }

  // Filter Initialization
  filter.Init(pos_k1, config_.init_std_pos, v_k1, config_.init_std_vel, att_k1, config_.init_std_att, config_.init_gb, config_.init_std_gb, config_.init_ab, config_.init_std_ab, config_.ARW, config_.VRW, imu_data_[i], imu_data_[i - 1], config_.is_gnss);

  // Epoch-wise Solve
  for (; i < imu_data_.size(); ++i) {
    if (k == gnss_data_.size() || n == pos_results_.size()) {
      break;
    }

    i_time = imu_data_[i][0];
    g_time = gnss_data_[k][0];
    r_time = pos_results_[n][0];
  
    if (g_time < i_time && fabs(g_time - i_time) > 1.0E-3) {
      std::vector<double> inter_imu = interpolate(imu_data_[i - 1], imu_data_[i], g_time);
      filter.ProcessData(inter_imu, gnss_data_[k++]);
    }
    else {
      filter.ProcessData(imu_data_[i], gnss_data_[(fabs(g_time - i_time) < 1.0E-3) ? k++ : k]);
    }

    v_k1 = filter.v_k2;
    pos_k1 = filter.pos_k2;
    att_k1 = filter.att_k2;

    Eigen::MatrixXd q_e = filter._insUpdate.q_b_e_ins;
    Eigen::MatrixXd tempPos = pos_k1 + q_e * ant_lever;
    posBL = XYZtoBLH(tempPos(0, 0), tempPos(1, 0), tempPos(2, 0));

    Eigen::MatrixXd Cen = E2L(posBL(0, 0), posBL(1, 0));
    Eigen::MatrixXd tempV = Cen * v_k1;
    Eigen::MatrixXd tempC = Cen * q_e;

    double pitchShow = 0.0, rollShow = 0.0, yawShow = 0.0;
    Mat2Euler(tempC, pitchShow, rollShow, yawShow);

    yawShow = Att2Azi(yawShow);

    result_file << std::fixed << std::setprecision(6) << imu_data_[i][0] << " "
      << std::fixed << std::setprecision(16) << Rad2Degree(posBL(0, 0)) << " " << Rad2Degree(posBL(1, 0)) << " " << posBL(2, 0) << " "
      << tempV(0, 0) << " " << tempV(1, 0) << " " << tempV(2, 0) << " "
      << Rad2Degree(yawShow) << " " << Rad2Degree(pitchShow) << " " << Rad2Degree(rollShow) << std::endl;

    if (g_time == r_time) {
      Eigen::MatrixXd pos_ref(3, 1);
      pos_ref << pos_results_[n][1], pos_results_[n][2], pos_results_[n][3];
      Eigen::MatrixXd NE = BLtoNE(posBL, pos_ref);
      double yaw_error = Rad2Degree(yawShow) - pos_results_[n][7];
      if (yaw_error < -180) {
        yaw_error += 360;
      }
      if (yaw_error > 180) {
        yaw_error -= 360;
      }

      diff_file << std::fixed << std::setprecision(6) << g_time << " "
        << std::fixed << std::setprecision(16) << NE(0, 0) << " " << NE(1, 0) << " " << NE(2, 0) << " "
        << tempV(0, 0) - pos_results_[n][4] << " " << tempV(1, 0) - pos_results_[n][5] << " " << tempV(2, 0) - pos_results_[n][6] << " "
        << yaw_error << " " << Rad2Degree(pitchShow) - pos_results_[n][8] << " "
        << Rad2Degree(rollShow) - pos_results_[n][9] << std::endl;
      n++;
    }
    else if (g_time > r_time) {
      n++;
    }

    ProgressBar(max_epoch, i);
  }

  result_file.close();
  diff_file.close();

  return true;
}

// Interpolate the IMU data 
std::vector<double> LooselyCoupled::interpolate(std::vector<double> last_imu_data, std::vector<double> imu_data, double time) {
  double duration = time - last_imu_data[0];
  std::vector<double> imu_data_inter;
  imu_data_inter.resize(7);
  imu_data_inter[0] = time;
  for (int i = 1; i < 7; i++) {
    imu_data_inter[i] = last_imu_data[i] + (imu_data[i] - last_imu_data[i]) * duration / (imu_data[0] - last_imu_data[0]);
  }

  return imu_data_inter;
}

void LooselyCoupled::ProgressBar(const int& max_num, const int& i) {
  int ten_percent = max_num / 20;

  if (i % ten_percent == 0) {
    double progress = ((double)i / max_num) * 100;

    std::cout << "Progress: " << progress << "%" << std::endl;
  }
}