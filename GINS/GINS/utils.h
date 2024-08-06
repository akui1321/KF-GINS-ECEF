/**
 * @file This file contains common definitions, mathematical utilities, and configuration
 * structures used throughout the loosely coupled integration of GNSS and IMU data.
 */

#pragma once

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

 // Define constants used throughout the system
#define a 6378137
#define e 0.08181919104
#define phi_0 30.5278800281
#define lambda_0 114.3556717814
#define pi 3.1415926535897932384626433832795028841971
#define R 6378137.0
#define g_0 9.7803267715
#define a_1 0.0052790414
#define a_2 0.0000232718
#define b_1 -0.000003087691891
#define b_2 0.0000000043977311
#define b_3 0.0000000000007211
#define R_WGS84 6378137.0  
#define F_WGS84 1.0/298.257223563

Eigen::MatrixXd BLtoNE(const Eigen::MatrixXd BL, const Eigen::MatrixXd BL_ref);

std::vector<std::vector<double>> enuToGeodetic(std::vector<std::vector<double>> NE);

Eigen::MatrixXd CrossProduct(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2);

double Magnitude(const Eigen::MatrixXd& mat);

void Mat2Euler(const Eigen::MatrixXd& C, double& pitch, double& roll, double& yaw);

Eigen::MatrixXd Euler2Mat(const double& pitch, const double& roll, const double& yaw);

Eigen::MatrixXd Vec2MatRight(const Eigen::MatrixXd& v);

Eigen::MatrixXd E2L(const double& B, const double& L);

// Quaternion class definition
class Quaternion {
public:
  double w, x, y, z;

  Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

  Quaternion operator*(const Quaternion& rhs) const;

  Quaternion normalize();

  void Quat2Mat(Eigen::MatrixXd& C);

  void Quat2Vec(Eigen::MatrixXd& C);
};

std::ostream& operator<<(std::ostream& os, const Quaternion& q);

double Azi2Att(const double& Azimuth);

double Att2Azi(const double& Attitude);

Eigen::MatrixXd Vec2Mat(const Eigen::MatrixXd& v);

Quaternion Vec2Quat(const Eigen::MatrixXd& C);

double BLH2g(const double& B, const double& L, const double& H);

Eigen::MatrixXd BLH2g_e(const double& B, const double& L, const double& g_BL);

Quaternion Euler2Quat(const double& yaw, const double& pitch, const double& roll);

void Quat2Euler(const Quaternion& q, double& yaw, double& pitch, double& roll);

double Rad2Degree(const double& rad);

Eigen::MatrixXd XYZtoBLH(const double& X, const double& Y, const double& Z);

void BLHtoXYZ(const double& B, const double& L, const double& H, double& X, double& Y, double& Z);

/**
 * @struct GINSConfig
 * A structure to hold configuration parameters for the GINS system.
 */
struct GINSConfig {
  std::string obsdata_IMUfile;       // OBSDATA source file for IMU
  std::string obsdata_GNSSfile;       // OBSDATA source file for GNSS
  std::string ref_file;
  std::string position_result_file; // Position result file path
  std::string position_diff_file;   // Position diff file path
  int is_gnss;
  int is_align;
  double init_pos[3], init_vel[3], init_std_pos[3], init_std_vel[3], init_std_att[3];
  double init_att[3];
  double init_gs[3], init_gb[3], init_as[3], init_ab[3], init_std_gs[3], init_std_gb[3], init_std_as[3], init_std_ab[3];
  double init_euler[3];
  double end_align_time;
  double lever[3];
  double ARW[3], VRW[3];

  GINSConfig()
  {
    obsdata_IMUfile = " ";
    obsdata_GNSSfile = " ";
    ref_file = " ";
    position_result_file = " ";
    position_diff_file = " ";
    is_gnss = 0;
    is_align = 0;
    end_align_time = 0.0;
    for (int i = 0; i < 3; i++) {
      init_pos[i] = init_vel[i]=init_att[i] = init_std_pos[i] = init_std_vel[i] = init_std_att[i] = init_euler[i] = lever[i] = 0.0;
      init_gs[i] = init_gb[i] = init_as[i] = init_ab[i] = init_std_gs[i] = init_std_gb[i] = init_std_as[i] = init_std_ab[i] = 0.0;
      ARW[i] = VRW[i] = 0;
    }
  }
};
