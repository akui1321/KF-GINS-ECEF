#include "ProcessConfig.h"

std::string Trim(const std::string& str)
{
  size_t first = str.find_first_not_of(" \t");
  size_t last = str.find_last_not_of(" \t");
  if (first == std::string::npos || last == std::string::npos)
    return "";
  return str.substr(first, (last - first + 1));
}

// Read the configuration file
void ReadConfig(const std::string& file_path, GINSConfig& config)
{
  std::ifstream file(file_path);
  std::string line;
  while (getline(file, line))
  {
    line = Trim(line);
    if (line.empty()) continue;

    std::string value = line.substr(line.find(":") + 1);
    value = Trim(value);

    if (line.find("OBSDATA SOURCE FILE(IMU):") != std::string::npos)
    {
      config.obsdata_IMUfile = value;
    }
    else if (line.find("OBSDATA SOURCE FILE(GNSS):") != std::string::npos)
    {
      config.obsdata_GNSSfile = value;
    }
    else if (line.find("REFERENCE RESULT FILE:") != std::string::npos)
    {
      config.ref_file = value;
    }
    else if (line.find("POSITION RESULT FILE:") != std::string::npos)
    {
      config.position_result_file = value;
    }
    else if (line.find("POSITION DIFF FILE:") != std::string::npos)
    {
      config.position_diff_file = value;
    }
    else if (line.find("IS GNSS:") != std::string::npos)
    {
      config.is_gnss = std::stoi(value);
    }
    else if (line.find("INITIAL ATT:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_att[0] >> config.init_att[1] >> config.init_att[2];
    }
    else if (line.find("INITIAL VEL:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_vel[0] >> config.init_vel[1] >> config.init_vel[2];
    }
    else if (line.find("INITIAL POS:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_pos[0] >> config.init_pos[1] >> config.init_pos[2];
    }
    else if (line.find("END TIME:") != std::string::npos)
    {
      config.end_align_time = std::stod(value);
    }
    else if (line.find("IS ALIGN:") != std::string::npos)
    {
      config.is_align = std::stoi(value);
    }
    else if (line.find("INITIAL STD ATT:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_std_att[0] >> config.init_std_att[1] >> config.init_std_att[2];
    }
    else if (line.find("INITIAL STD VEL:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_std_vel[0] >> config.init_std_vel[1] >> config.init_std_vel[2];
    }
    else if (line.find("INITIAL STD POS:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_std_pos[0] >> config.init_std_pos[1] >> config.init_std_pos[2];
    }
    else if (line.find("INITIAL GB:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_gb[0] >> config.init_gb[1] >> config.init_gb[2];
    }
    else if (line.find("INTIIAL AB:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_ab[0] >> config.init_ab[1] >> config.init_ab[2];
    }
    else if (line.find("INITIAL GS:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_gs[0] >> config.init_gs[1] >> config.init_gs[2];
    }
    else if (line.find("INITIAL AS:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_as[0] >> config.init_as[1] >> config.init_as[2];
    }
    else if (line.find("INTTIAL STD GB:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_std_gb[0] >> config.init_std_gb[1] >> config.init_std_gb[2];
    }
    else if (line.find("INTIIAL STD AB:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_std_ab[0] >> config.init_std_ab[1] >> config.init_std_ab[2];
    }
    else if (line.find("INITIAL STD GS:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_std_gs[0] >> config.init_std_gs[1] >> config.init_std_gs[2];
    }
    else if (line.find("INITIAL STD AS:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.init_std_as[0] >> config.init_std_as[1] >> config.init_std_as[2];
    }
    else if (line.find("LEVER:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.lever[0] >> config.lever[1] >> config.lever[2];
    }
    else if (line.find("ARW:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.ARW[0] >> config.ARW[1] >> config.ARW[2];
    }
    else if (line.find("VRW:") != std::string::npos) {
      std::string values = line.substr(line.find(":") + 1);
      std::istringstream iss(values);
      iss >> config.VRW[0] >> config.VRW[1] >> config.VRW[2];
    }
  }
}