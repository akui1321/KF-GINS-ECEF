#include "utils.h"

// Conversion of latitude and longitude to northeast coordinates
Eigen::MatrixXd BLtoNE(const Eigen::MatrixXd BL,const Eigen::MatrixXd BL_ref_) {
  Eigen::MatrixXd BL_ref(3, 1);

  BL_ref(0, 0) = BL_ref_(0, 0) * pi / 180.0;
  BL_ref(1, 0) = BL_ref_(1, 0) * pi / 180.0;
  BL_ref(2, 0) = BL_ref_(2, 0);

  Eigen::MatrixXd NE(3, 1);
  NE(2, 0) = BL(2, 0) - BL_ref(2, 0);

  double R_M = a * (1 - e * e) / (1 - e * e * sin(BL(0, 0) / 180.0 * pi) * sin(BL(0, 0) / 180.0 * pi)) / sqrt(1 - e * e * sin(BL(0, 0) / 180.0 * pi) * sin(BL(0, 0) / 180.0 * pi));
  double R_N = a / sqrt(1 - e * e * sin(BL(0, 0) / 180.0 * pi) * sin(BL(0, 0) / 180.0 * pi));
  NE(1, 0) = (BL(1, 0) - BL_ref(1, 0)) / 180.0 * pi * (R_N + BL(2, 0)) * cos(BL(0, 0) / 180.0 * pi);
  NE(0, 0) = (BL(0, 0) - BL_ref(0, 0)) / 180.0 * pi * (R_M + BL(2, 0));

  return NE;
}

// ENU to BLH
std::vector<std::vector<double>> enuToGeodetic(std::vector<std::vector<double>> NE) {
    std::vector<std::vector<double>> BL;
    BL = NE;

    double latRef = 30.5278800644;
    double lonRef = 114.3556717508;
    double hRef = 21.895;

    for (auto& row : BL) {
        row[2] = latRef + (row[2] / R) * (180.0 / pi);
        row[1] = lonRef + (row[1] / (R * cos(latRef / 180.0 * pi))) * (180.0 / pi);
        row[3] = hRef + row[3];
    }

    return BL;
}

// Skew-symmetric matrix
Eigen::MatrixXd CrossProduct(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2) {
    Eigen::MatrixXd temp(3, 3);
    temp.fill(0);
    temp(0, 1) = -mat1(2, 0);
    temp(0, 2) = mat1(1, 0);
    temp(1, 0) = mat1(2, 0);
    temp(1, 2) = -mat1(0, 0);
    temp(2, 0) = -mat1(1, 0);
    temp(2, 1) = mat1(0, 0);

    return temp * mat2;
}

// Magnitude of the vector
double Magnitude(const Eigen::MatrixXd& mat) {
    return sqrt(mat(0, 0) * mat(0, 0) + mat(1, 0) * mat(1, 0) + mat(2, 0) * mat(2, 0));
}

// Rotation Matrix to Euler Angles
void Mat2Euler(const Eigen::MatrixXd& C, double& pitch, double& roll, double& yaw) {
  pitch = asin(C(2, 1));
  roll = atan2(-C(2, 0), C(2, 2));
  yaw = atan2(-C(0, 1), C(1, 1));
}

// Euler Angles to Rotation Matrix
Eigen::MatrixXd Euler2Mat(const double& pitch, const double& roll, const double& yaw) {
  Eigen::MatrixXd C(3, 3);
  C << cos(roll) * cos(yaw) - sin(roll) * sin(pitch) * sin(yaw), -sin(yaw) * cos(pitch), sin(roll)* cos(yaw) + cos(roll) * sin(pitch) * sin(yaw),
    sin(roll)* sin(pitch)* cos(yaw) + cos(roll) * sin(yaw), cos(yaw)* cos(pitch), sin(roll)* sin(yaw) - cos(roll) * sin(pitch) * cos(yaw),
    -sin(roll) * cos(pitch), sin(pitch), cos(roll)* cos(pitch);

  return C;
}

// ECEF to Local
Eigen::MatrixXd E2L(const double& B, const double& L) {
  Eigen::MatrixXd C(3, 3);
  C(0, 0) = -sin(L);
  C(0, 1) = cos(L);
  C(0, 2) = 0;
  C(1, 0) = -sin(B) * cos(L);
  C(1, 1) = -sin(B) * sin(L);
  C(1, 2) = cos(B);
  C(2, 0) = cos(B) * cos(L);
  C(2, 1) = cos(B) * sin(L);
  C(2, 2) = sin(B);

  return C;
}

// Quaternion Multiplication
Quaternion Quaternion::operator*(const Quaternion& rhs) const {
  return Quaternion(
    w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
    w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
    w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
    w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
  );
}

// Quaternion normalization
Quaternion Quaternion::normalize() {
  double norm = std::sqrt(w * w + x * x + y * y + z * z);
  double w_norm = 0.0;
  double x_norm = 0.0;
  double y_norm = 0.0;
  double z_norm = 0.0;
  if (norm > 0) {
    w_norm = w / norm;
    x_norm = x / norm;
    y_norm = y / norm;
    z_norm = z / norm;
  }
  return Quaternion(
    w_norm,
    x_norm,
    y_norm,
    z_norm
  );
}

// Quaternion to Rotation Matrix
void Quaternion::Quat2Mat(Eigen::MatrixXd& C) {
  C(0, 0) = w * w + x * x - y * y - z * z;
  C(0, 1) = 2 * (x * y + w * z);
  C(0, 2) = 2 * (x * z - w * y);
  C(1, 0) = 2 * (x * y - w * z);
  C(1, 1) = w * w - x * x + y * y - z * z;
  C(1, 2) = 2 * (y * z + w * x);
  C(2, 0) = 2 * (x * z + w * y);
  C(2, 1) = 2 * (y * z - w * x);
  C(2, 2) = w * w - x * x - y * y + z * z;
}

// Quaternion to Rotation Vector
void Quaternion::Quat2Vec(Eigen::MatrixXd& C) {
  double zeta = 2 * atan2(sqrt(x * x + y * y + z * z), w) / sqrt(x * x + y * y + z * z);
  C(0, 0) = x * zeta;
  C(1, 0) = y * zeta;
  C(2, 0) = z * zeta;
}

// Quaternion Output
std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
  os << q.w << " + " << q.x << "i + " << q.y << "j + " << q.z << "k";
  return os;
}

// Azimuth to Attitude
double Azi2Att(const double& Azimuth) {
  double Attitude = 0.0;
  if (Azimuth <= pi) {
    Attitude = -Azimuth;
  }
  if (Azimuth > pi) {
    Attitude = 2 * pi - Azimuth;
  }

  return Azimuth;
}

// Attitude to Azimuth
double Att2Azi(const double& Attitude) {
  double Azimuth = 0.0;
  if (Attitude < 0) {
    Azimuth = -Attitude;
  }
  if (Attitude >= 0) {
    Azimuth = 2 * pi - Attitude;
  }

  return Azimuth;
}

// Rotation Vector to Rotation Matrix
Eigen::MatrixXd Vec2Mat(const Eigen::MatrixXd& v) {
  Eigen::Matrix3d identityMatrix = Eigen::Matrix3d::Identity();
  Eigen::MatrixXd C(3, 3);
  C = identityMatrix - sin(Magnitude(v)) / Magnitude(v) * CrossProduct(v, identityMatrix) + (1 - cos(Magnitude(v))) / Magnitude(v) / Magnitude(v) * CrossProduct(v, identityMatrix) * CrossProduct(v, identityMatrix);

  return C;
}

Eigen::MatrixXd Vec2MatRight(const Eigen::MatrixXd& v) {
  Eigen::Matrix3d identityMatrix = Eigen::Matrix3d::Identity();
  Eigen::MatrixXd C(3, 3);
  C = identityMatrix + sin(Magnitude(v)) / Magnitude(v) * CrossProduct(v, identityMatrix) + (1 - cos(Magnitude(v))) / Magnitude(v) / Magnitude(v) * CrossProduct(v, identityMatrix) * CrossProduct(v, identityMatrix);

  return C;
}

// Calculate g
double BLH2g(const double& B, const double& L, const double& H) {
  return g_0 * (1 + a_1 * pow(sin(B), 2) + a_2 * pow(sin(B), 4)) + (b_1 + b_2 * pow(sin(B), 2)) * H + b_3 * pow(H, 2);
}

// Calculate g_e
Eigen::MatrixXd BLH2g_e(const double& B, const double& L, const double& g_BL) {
  Eigen::MatrixXd g_e(3, 1);
  g_e(0, 0) = cos(L) * cos(B) * (-g_BL);
  g_e(1, 0) = sin(L) * cos(B) * (-g_BL);
  g_e(2, 0) = sin(B) * (-g_BL);

  return g_e;
}

// Rad to Degree
double Rad2Degree(const double& rad) {
  return rad / pi * 180;
}

// Rotation Vector to Quaternion
Quaternion Vec2Quat(const Eigen::MatrixXd& C) {
  double real = cos(0.5 * Magnitude(C));
  Eigen::Vector3d q = (0.5 * C) * sin(0.5 * Magnitude(C)) / (0.5 * Magnitude(C));
  return Quaternion{ real, q(0), q(1), q(2) };
}

// Euler Angles to Quaternion
Quaternion Euler2Quat(const double& yaw, const double& pitch, const double& roll) {
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Quaternion q(1, 0, 0, 0);
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

// Quaternion to Euler Angles
void Quat2Euler(const Quaternion& q, double& yaw, double& pitch, double& roll) {
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  roll = atan2(sinr_cosp, cosr_cosp);

  double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
  double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
  pitch = 2 * atan2(sinp, cosp) - pi / 2;

  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  yaw = atan2(siny_cosp, cosy_cosp);
  while (yaw < 0) {
    yaw += 2 * pi;
  }
  while (yaw > 2 * pi) {
    yaw -= 2 * pi;
  }
}

//Convert XYZ to BLH
Eigen::MatrixXd XYZtoBLH(const double& X, const double& Y, const double& Z) {
  double a_ = R_WGS84;
  double f = F_WGS84;
  double e_ = sqrt(2 * f - f * f);
  double B0 = 0.0;
  double L = atan2(Y, X);

  B0 = atan2(Z, sqrt(pow(X, 2) + pow(Y, 2)));
  double B = B0;
  double N1;

  //Iterative Calculation of B
  do
  {
    B0 = B;
    N1 = a_ / sqrt(1 - e_ * e_ * sin(B) * sin(B));
    B = atan2((Z + N1 * sin(B) * pow(e_, 2)), sqrt(pow(X, 2) + pow(Y, 2)));
  } while (abs(B - B0) > 0.000001);

  N1 = a_ / sqrt(1 - e_ * e_ * sin(B) * sin(B));
  double H = sqrt(pow(X, 2) + pow(Y, 2)) / cos(B) - N1;

  Eigen::MatrixXd posBL(3, 1);
  posBL(0, 0) = B;
  posBL(1, 0) = L;
  posBL(2, 0) = H;

  return posBL;
}

// Convert BLH to XYZ
void BLHtoXYZ(const double& B, const double& L, const double& H, double& X, double& Y, double& Z) {
  double a_ = R_WGS84;
  double f = F_WGS84;
  double e_ = sqrt(2 * f - f * f);
  double B_tran = B / 180.0 * pi;
  double L_tran = L / 180.0 * pi;
  double N1 = a_ / sqrt(1 - e_ * e_ * sin(B_tran) * sin(B_tran));
  X = (N1 + H) * cos(B_tran) * cos(L_tran);
  Y = (N1 + H) * cos(B_tran) * sin(L_tran);
  Z = (N1 * (1 - e_ * e_) + H) * sin(B_tran);
}