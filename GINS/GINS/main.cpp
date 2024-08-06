#include "IMUDataReader.h"
#include "GNSSDataReader.h"
#include "utils.h"
#include "ProcessConfig.h"
#include "LooselyCoupled.h"

/**
 * \brief The main entry point of the program.
 *
 * This function first reads system settings from the configuration file, then reads IMU and GNSS data files.
 * Next, it reads the reference result data and finally calls the process method of the LooselyCoupled class to perform data processing.
 *
 * \param argc The number of command-line arguments (unused).
 * \param argv The array of command-line arguments (unused).
 * \return 0.
 */
int main() {
  std::cout << "***************************************" << std::endl
    << "GINS - GNSS-INS Loosely Coupled System" << std::endl
    << "***************************************" << std::endl
    << std::endl;

  std::cout << "Start Loading Files..." << std::endl;
  // Read Configure File
  GINSConfig config;
  ReadConfig("D:\\GINS\\GINS\\GINS\\friend.cfg", config);

  // Read IMU File
  std::vector<std::vector<double>> imu_data;
  IMUDataReader IMUData(config.obsdata_IMUfile);
  if (IMUData.readData()) {
    imu_data = IMUData.rawData;
  }

  // Read GNSS File
  std::vector<std::vector<double>> gnss_data;
  GNSSDataReader GNSSData(config.obsdata_GNSSfile);
  if (GNSSData.loadData()) {
    gnss_data = GNSSData.getData();
  }

  // Read Reference Result
  std::vector<std::vector<double>> pos_results;
  GNSSDataReader POSResults(config.ref_file);
  if (POSResults.loadData()) {
    pos_results = POSResults.getData();
  }

  std::cout << "Files Loading Complete!" << std::endl;
  std::cout << "Start to Solve..." << std::endl;

  // Solve
  LooselyCoupled LooseGINS(imu_data, gnss_data, pos_results, config);
  if (LooseGINS.process()) {
    std::cout << "Finished!" << std::endl;
  }
  else {
    std::cout << "Failed to solve!" << std::endl;
  }

  return 0;
}
