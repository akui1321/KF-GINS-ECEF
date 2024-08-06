#ifndef GINS_FILTER_H
#define GINS_FILTER_H

#include "Updating.h"
#include "utils.h"

/**
 * @class GinsFilter
 * A class that represents the GINS (GNSS/INS) filter for loosely coupled integration of GNSS and IMU data.
 */
class GinsFilter {
public:
  GinsFilter()
    : _gb{ 0.0, 0.0, 0.0 }, _ab{ 0.0, 0.0, 0.0 },
    _std_gb{ 0.0, 0.0, 0.0 }, _std_ab{ 0.0, 0.0, 0.0 },
    _pos_std{ 0.0, 0.0, 0.0 }, _ARW{ 0.0, 0.0, 0.0 }, _VRW{ 0.0, 0.0, 0.0 },
    _lever{ 0.0, 0.0, 0.0 } {
  }
  void Init(const Eigen::MatrixXd& pos, const double* pos_std, const Eigen::MatrixXd& vel, const double* vel_std, const Eigen::MatrixXd& att, const double* att_std,
    const double* gb, const double* gb_std, const double* ab, const double* ab_std, const double* ARW, const double* VRW,
    const std::vector<double>& imu_data, const std::vector<double>& last_imu_data, const int& is_gnss_);
  void ProcessData(std::vector<double>& imu_data, const std::vector<double>& gnss_data);
  void SetLeverArm(const double* lever);

  Eigen::MatrixXd att_k2;
  Eigen::MatrixXd pos_k2;
  Eigen::MatrixXd v_k2;
  Updating _insUpdate;

private:
  Eigen::MatrixXd BuildF(const std::vector<double>& imu_data, double dt);
  Eigen::MatrixXd BuildQ(const Eigen::MatrixXd& F, double dt);
  Eigen::MatrixXd BuildH(double dt);
  Eigen::MatrixXd Buildz(const std::vector<double>& imu_data, const std::vector<double>& gnss_data);
  Eigen::MatrixXd BuildR(const std::vector<double>& gnss_data);
  void CorrectState(double dt);
  void InsCorrect(const Eigen::Vector3d& dp, const Eigen::Vector3d& dv, const Eigen::MatrixXd& dq);
  void Compensate(std::vector<double>& imu_data, const double* gb, const double* ab);
  void KFPredict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q, const double& dt, Eigen::MatrixXd& _x, Eigen::MatrixXd& _P);
  void KFUpdate(const Eigen::MatrixXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& _R_, const double& dt, Eigen::MatrixXd& _x, Eigen::MatrixXd& _P);
  Eigen::MatrixXd get_rpy_q();
  
  double _gb[3], _ab[3], _std_gb[3], _std_ab[3], _pos_std[3], _ARW[3], _VRW[3], _lever[3];
  Eigen::MatrixXd _x, _P, _Qc;
  Eigen::MatrixXd omega_ie_e;
  std::vector<double> _last_imu_data;
  int is_gnss;
};

#endif