#include "IMUDataReader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>

IMUDataReader::IMUDataReader(const std::string &filename) : filename(filename) {}

/**
 * @brief Reads IMU data from the file specified in the constructor.
 *
 * This function opens the IMU data file and reads it line by line. It processes each line
 * to extract the required data, which includes the timestamp and values for the IMU's
 * gyroscope and accelerometer measurements. The extracted data is then converted to a
 * standardized format and stored in the 'rawData' vector. The function skips lines that
 * are empty or do not contain the expected amount of data.
 *
 * @return true if the data was successfully read and processed, false if the file could not be opened.
 */
bool IMUDataReader::readData() {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;

    size_t asteriskPos = line.find('*');
    if (asteriskPos != std::string::npos) {
      line = line.substr(0, asteriskPos);
    }

    std::stringstream ss(line);
    std::string segment;
    std::vector<std::string> segments;

    // Split the line by commas and semicolons
    while (std::getline(ss, segment, ',')) {
      size_t pos = segment.find(';');
      if (pos != std::string::npos) {
        segments.push_back(segment.substr(0, pos));
        segments.push_back(segment.substr(pos + 1));
      }
      else {
        segments.push_back(segment);
      }
    }

    if (segments.size() < 12) continue; // Ensure there's enough data in the line

    // Extract the required columns
    double timestamp = std::stod(segments[4]);
    double gyroY = std::stod(segments[11]) * 1e-9 * 200;
    double gyroX = -std::stod(segments[10]) * 1e-9 * 200;
    double gyroZ = -std::stod(segments[9]) * 1e-9 * 200;
    double accelY = std::stod(segments[8]) * 2e-8 * 200;
    double accelX = -std::stod(segments[7]) * 2e-8 * 200;
    double accelZ = -std::stod(segments[6]) * 2e-8 * 200;

    rawData.push_back({ timestamp, gyroX, gyroY, gyroZ, accelX, accelY, accelZ });
  }

  file.close();
  return true;
}
