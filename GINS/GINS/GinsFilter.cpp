#include "GinsFilter.h"
#include "utils.h"

/**
 * @brief Initializes the GINS filter with a set of initial conditions and standard deviations.
 *
 * This method sets up the initial state estimate, covariance matrix, and other parameters required for
 * the operation of the GINS filter. It takes various inputs including position, velocity, attitude,
 * and their respective standard deviations. It also initializes the gyroscope and accelerometer biases
 * and their uncertainties, as well as the angular and velocity random walk values.
 */
void GinsFilter::Init(const Eigen::MatrixXd& pos, const double* pos_std, const Eigen::MatrixXd& vel, const double* vel_std, const Eigen::MatrixXd& att, const double* att_std,
  const double* gb, const double* gb_std, const double* ab, const double* ab_std, const double* ARW, const double* VRW,
  const std::vector<double>& imu_data, const std::vector<double>& last_imu_data, const int& is_gnss_) {
  _last_imu_data = last_imu_data;
  _x = Eigen::MatrixXd(15, 1);
  _P = Eigen::MatrixXd(15, 15);
  _Qc = Eigen::MatrixXd(6, 6);
  omega_ie_e = Eigen::MatrixXd(3, 1);
  is_gnss = is_gnss_;
  _insUpdate.Init(_last_imu_data, imu_data, vel, pos, att);

  for (int i = 0; i < 3; ++i) {
    _gb[i] = gb[i];
    _ab[i] = ab[i];
    _std_gb[i] = gb_std[i];
    _std_ab[i] = ab_std[i];
    _pos_std[i] = pos_std[i];
    _VRW[i] = VRW[i];
    _ARW[i] = ARW[i];
  }

  _x.fill(0);

  Eigen::VectorXd std(15);
  std.setZero();
  std.segment(0, 3) = Eigen::Vector3d(pos_std[0], pos_std[1], pos_std[2]);
  std.segment(3, 3) = Eigen::Vector3d(vel_std[0], vel_std[1], vel_std[2]);
  std.segment(6, 3) = Eigen::Vector3d(att_std[0], att_std[1], att_std[2]);
  std.segment(9, 3) = Eigen::Vector3d(ab_std[0], ab_std[1], ab_std[2]);
  std.segment(12, 3) = Eigen::Vector3d(gb_std[0], gb_std[1], gb_std[2]);
  _P = std.asDiagonal();
  _P *= _P;

  _Qc.fill(0);

  Eigen::VectorXd vrw_std(3);
  vrw_std.setZero();
  Eigen::VectorXd arw_std(3);
  arw_std.setZero();
  vrw_std.segment(0, 3) = Eigen::Vector3d(_VRW[0]* _VRW[0], _VRW[1]* _VRW[1], _VRW[2]* _VRW[2]);
  arw_std.segment(0, 3) = Eigen::Vector3d(_ARW[0]* _ARW[0], _ARW[1]* _ARW[1], _ARW[2]* _ARW[2]);
  _Qc.block(0, 0, 3, 3) = vrw_std.asDiagonal();
  _Qc.block(3, 3, 3, 3) = arw_std.asDiagonal();

  omega_ie_e(0, 0) = 0;
  omega_ie_e(1, 0) = 0;
  omega_ie_e(2, 0) = omega_e;
}

/**
 * @brief Sets the lever arm parameters for the GINS filter.
 * @param lever A pointer to an array of three doubles representing the lever arm values respectively.
 */
void GinsFilter::SetLeverArm(const double* lever) {
  for (int i = 0; i < 3; ++i) {
    _lever[i] = lever[i];
  }
}

/**
 * @brief Constructs the state transition matrix F for the GINS filter.
 *
 * The state transition matrix F is a key component of the Kalman filter equations,
 * representing the predicted evolution of the system state over a small time interval dt.
 * This method calculates F based on the latest IMU data and the Earth's rotation vector.
 *
 * @param imu_data A vector containing the latest IMU measurements including
 *                  gyroscope and accelerometer values.
 * @param dt The time interval over which the state transition is calculated.
 * @return The 15x15 state transition matrix F.
 */
Eigen::MatrixXd GinsFilter::BuildF(const std::vector<double>& imu_data, double dt) {
  Eigen::MatrixXd F(15, 15);
  F.fill(0);
  Eigen::MatrixXd acc(3, 1), gyr(3, 1);
  for (int i = 0; i < 3; i++) {
    acc(i, 0) = imu_data[i + 4];
    gyr(i, 0) = imu_data[i + 1];
  }

  Eigen::MatrixXd q = get_rpy_q();

  auto I_33 = Eigen::Matrix3d::Identity();

  F.block(0, 3, 3, 3) = I_33;
  F.block(3, 3, 3, 3) = -2 * CrossProduct(omega_ie_e, I_33);
  F.block(3, 6, 3, 3) = CrossProduct(q * acc, I_33);
  F.block(3, 9, 3, 3) = q;
  F.block(6, 6, 3, 3) = -CrossProduct(omega_ie_e, I_33);
  F.block(6, 12, 3, 3) = -q;

  return F;
}

/**
 * @brief Constructs the observation matrix H for the GINS filter.
 *
 * The observation matrix H maps the state vector to the observation vector and is used in the
 * measurement update step of the Kalman filter. This method calculates H based on the current
 * lever arm values and the orientation represented by the roll-pitch-yaw (RPY).
 *
 * @param dt The time interval over which the observation matrix is calculated.
 * @return The 3x15 observation matrix H.
 */
Eigen::MatrixXd GinsFilter::BuildH(double dt) {
  Eigen::MatrixXd H(3, 15);
  H.fill(0);
  Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();

  Eigen::MatrixXd lb(3, 1);
  for (int i = 0; i < 3; i++) {
    lb(i, 0) = _lever[i];
  }

  Eigen::MatrixXd lever_e = get_rpy_q() * lb;

  H.block(0, 0, 3, 3) = I_33;
  H.block(0, 6, 3, 3) = CrossProduct(lever_e, I_33);

  return H;
}

/**
 * @brief Calculates the process noise covariance matrix Q for the GINS filter.
 *
 * The process noise covariance matrix Q represents the uncertainty in the system's process
 * model. It is used in the time update step of the Kalman filter to account for process noise.
 * This method computes Q based on the state transition matrix F, the process noise covariance
 * _Qc, and the time interval dt.
 *
 * @param F The state transition matrix.
 * @param dt The time interval over which the process noise is considered.
 * @return The 15x15 process noise covariance matrix Q.
 */
Eigen::MatrixXd GinsFilter::BuildQ(const Eigen::MatrixXd& F, double dt) {
  Eigen::MatrixXd Q(15, 15);
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(15, 15) + F * dt;
  Eigen::MatrixXd q(15, 15);
  Eigen::MatrixXd G(15, 6);
  Eigen::MatrixXd I_33 = Eigen::Matrix3d::Identity();
  G.fill(0);
  q.fill(0);

  G.block(3, 0, 3, 3) = get_rpy_q();
  G.block(6, 3, 3, 3) = -get_rpy_q();

  q = G * _Qc * G.transpose() * dt;

  Q = (Phi * q * Phi.transpose() + q) * 0.5;

  return Q;
}

/**
 * @brief Constructs the measurement vector z by comparing IMU-based position with GNSS data.
 *
 * This function creates a measurement vector z that represents the difference between the
 * estimated position from the IMU (after considering the lever arm) and the position provided
 * by the GNSS system. This difference is used in the Kalman filter to correct the state
 * estimation.
 *
 * @param imu_data A vector containing the latest IMU measurements.
 * @param gnss_data A vector containing the latest GNSS position data.
 * @return The 3x1 measurement vector z representing the difference in position.
 */
Eigen::MatrixXd GinsFilter::Buildz(const std::vector<double>& imu_data, const std::vector<double>& gnss_data) {
  Eigen::MatrixXd z(3, 1);
  z.fill(0);
  Eigen::MatrixXd lb(3, 1);
  for (int i = 0; i < 3; i++) {
    lb(i, 0) = _lever[i];
  }

  Eigen::MatrixXd pos_imu(3, 1), pos_gnss(3, 1);

  pos_imu = _insUpdate.pos_k2 + get_rpy_q() * lb;
  for (int i = 0; i < 3; i++) {
    pos_gnss(i, 0) = gnss_data[i + 1];
  }

  z = pos_imu - pos_gnss;

  return z;
}

/**
 * @brief Constructs the measurement noise covariance matrix R for GNSS data.
 *
 * This function creates the measurement noise covariance matrix R based on the provided
 * GNSS data, which includes the variances of the GNSS position measurements along each axis.
 * The matrix R is used in the Kalman filter to account for the uncertainty in the measurement process.
 *
 * @param gnss_data A vector containing the latest GNSS data.
 * @return The 3x3 measurement noise covariance matrix R.
 */
Eigen::MatrixXd GinsFilter::BuildR(const std::vector<double>& gnss_data) {
  Eigen::MatrixXd _R_(3, 3);
  Eigen::Vector<double, 3> R1{ pow(gnss_data[4], 2), pow(gnss_data[5], 2), pow(gnss_data[6], 2) };
  _R_ = R1.asDiagonal();
  return _R_;
}

/**
 * @brief Processes IMU data and GNSS corrections in a single time step of the filter.
 *
 * This method performs the core functionality of the GINS filter for a given time step.
 * It starts by calculating the time difference dt since the last IMU data was processed.
 * The IMU data is compensated for sensor biases, and the INS (Inertial Navigation System)
 * is updated with the new data. If a GNSS data point is available within the same time step,
 * a Kalman filter prediction and update step is performed to correct the INS solution using
 * the GNSS information.
 *
 * @param imu_data A reference to a vector containing the latest IMU measurements.
 *                 The first element is expected to be the timestamp.
 * @param gnss_data A const reference to a vector containing the latest GNSS measurements.
 *                  The first element is expected to be the timestamp.
 */
void GinsFilter::ProcessData(std::vector<double>& imu_data, const std::vector<double>& gnss_data) {
  double dt = imu_data[0] - _last_imu_data[0];

  Compensate(imu_data, _gb, _ab);

  _insUpdate.calculate(_last_imu_data, imu_data);

  att_k2 = _insUpdate.att_k2;
  v_k2 = _insUpdate.v_k2;
  pos_k2 = _insUpdate.pos_k2;

  if (is_gnss) {
    Eigen::MatrixXd F = BuildF(imu_data, dt);

    Eigen::MatrixXd Q = BuildQ(F, dt);

    // Predict
    KFPredict(F, Q, dt, _x, _P);

    // Update
    if (fabs(imu_data[0] - gnss_data[0]) < 1.0E-3) {
      Eigen::MatrixXd z = Buildz(imu_data, gnss_data);

      Eigen::MatrixXd H = BuildH(dt);

      Eigen::MatrixXd _R_ = BuildR(gnss_data);

      KFUpdate(z, H, _R_, dt, _x, _P);

      CorrectState(dt);
    }
  }

  att_k2 = _insUpdate.att_k2;
  v_k2 = _insUpdate.v_k2;
  pos_k2 = _insUpdate.pos_k2;

  _last_imu_data = imu_data;
}

/**
 * @brief Applies corrections to the state estimate based on the current state increments.
 *
 * This method corrects the current state estimate by applying the increments in position (dp),
 * velocity (dv), and orientation (dphi) that are stored in the state vector _x. It also updates
 * the gyroscope (gb) and accelerometer (ab) biases using the increments in the state vector
 * over the time interval dt.
 *
 * @param dt The time interval over which the corrections are applied. This is used to scale
 *            the bias increments before updating the _gb and _ab values.
 */
void GinsFilter::CorrectState(double dt) {
  Eigen::Vector3d dp(_x(0), _x(1), _x(2));
  Eigen::Vector3d dv(_x(3), _x(4), _x(5));
  Eigen::MatrixXd dphi(3, 1);
  for (int i = 0; i < 3; i++) {
    dphi(i, 0) = _x(i + 6);
  }

  InsCorrect(dp, dv, dphi);

  Eigen::Vector3d dgb(_x(12), _x(13), _x(14));
  Eigen::Vector3d dab(_x(9), _x(10), _x(11));
  for (int i = 0; i < 3; ++i) {
    _gb[i] += dgb(i) * dt;
    _ab[i] += dab(i) * dt;
  }
  
  _x.fill(0);
}

/**
 * @brief Compensates the IMU data for sensor biases.
 *
 * This function adjusts the raw IMU data to compensate for known biases in the
 * gyroscope (angular rate) and accelerometer (acceleration) measurements.
 *
 * @param imu_data A reference to a vector containing the raw IMU measurements.
 * @param gb A pointer to an array containing the gyroscope biases.
 * @param ab A pointer to an array containing the accelerometer biases.
 */
void GinsFilter::Compensate(std::vector<double>& imu_data, const double* gb, const double* ab) {
  for (int i = 1; i < 4; i++) {
    imu_data[i] = imu_data[i] - gb[i - 1];
  }
  for (int i = 4; i < 7; i++) {
    imu_data[i] = imu_data[i] - ab[i - 4];
  }
}

/**
 * @brief Performs the Kalman filter prediction step.
 *
 * This function predicts the state estimate and the state covariance for the next time step
 * using the state transition matrix F and the process noise covariance Q.
 *
 * @param F The state transition matrix.
 * @param Q The process noise covariance matrix.
 * @param dt The time interval over which the prediction is made.
 * @param _x A reference to the current state estimate vector.
 * @param _P A reference to the current state covariance matrix.
 */
void GinsFilter::KFPredict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q, const double& dt, Eigen::MatrixXd& _x, Eigen::MatrixXd& _P) {
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(15, 15) + F * dt;

  _x = Phi * _x;
  _P = Phi * _P * Phi.transpose() + Q;
}

/**
 * @brief Performs the Kalman filter update step.
 *
 * This function updates the state estimate and the state covariance using the measurement vector z,
 * the observation matrix H, and the measurement noise covariance _R_.
 *
 * @param z The measurement vector.
 * @param H The observation matrix.
 * @param _R_ The measurement noise covariance matrix.
 * @param dt The time interval over which the update is made (not used in this function).
 * @param _x A reference to the current state estimate vector.
 * @param _P A reference to the current state covariance matrix.
 */
void GinsFilter::KFUpdate(const Eigen::MatrixXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& _R_, const double& dt, Eigen::MatrixXd& _x, Eigen::MatrixXd& _P) {
  Eigen::MatrixXd K = _P * H.transpose() * (H * _P * H.transpose() + _R_).inverse();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
  _x = _x + K * (z - H * _x);
  _P = (I - K * H) * _P * (I - K * H).transpose() + K * _R_ * K.transpose();
}

/**
 * @brief Corrects the INS state.
 */
void GinsFilter::InsCorrect(const Eigen::Vector3d& dp, const Eigen::Vector3d& dv, const Eigen::MatrixXd& dq) {
  for (int i = 0; i < 3; i++) {
    _insUpdate.pos_k2(i, 0) -= dp(i);
    _insUpdate.v_k2(i, 0) -= dv(i);
  }

  Eigen::MatrixXd att(3, 3);
  Eigen::MatrixXd posBL = XYZtoBLH(_insUpdate.pos_k2(0, 0), _insUpdate.pos_k2(1, 0), _insUpdate.pos_k2(2, 0));

  Eigen::MatrixXd Cel = E2L(posBL(0, 0), posBL(1, 0));
  att = Cel * ((Eigen::Matrix3d::Identity() + CrossProduct(dq, Eigen::Matrix3d::Identity())) * get_rpy_q());
  Mat2Euler(att, _insUpdate.att_k2(1, 0), _insUpdate.att_k2(2, 0), _insUpdate.att_k2(0, 0));
  _insUpdate.q_b_e_ins = ((Eigen::Matrix3d::Identity() + CrossProduct(dq, Eigen::Matrix3d::Identity())) * get_rpy_q());
}

Eigen::MatrixXd GinsFilter::get_rpy_q() {
  return _insUpdate.q_b_e_ins;
}