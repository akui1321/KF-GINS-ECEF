#ifndef IMU_DATA_READER_H
#define IMU_DATA_READER_H

#include <string>
#include <vector>

/**
 * @class IMUDataReader
 * A class designed to read IMU (Inertial Measurement Unit) data from a file.
 */
class IMUDataReader {
public:
  IMUDataReader(const std::string &filename);

  bool readData();
  
  std::vector<std::vector<double>> rawData;

private:
  std::string filename;

  std::vector<double> averages;
};

#endif 
