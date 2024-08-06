#include "GNSSDataReader.h"

GNSSDataReader::GNSSDataReader(const std::string& filename) : filename(filename), loaded(false) {}

/**
 * @brief Loads GNSS data from the file specified in the constructor.
 *
 * This function opens the file, reads the data line by line, and processes each line
 * using the parseLine method. It skips the header lines and starts parsing from the
 * third line. Once the file is read completely, it closes the file and sets the
 * 'loaded' flag to true, indicating that the data has been successfully loaded.
 *
 * @return true if the data was successfully loaded, false if the file could not be opened.
 */
bool GNSSDataReader::loadData() {
  std::ifstream file(filename);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  std::getline(file, line); // Skip the first header line
  std::getline(file, line); // Skip the second header line

  while (std::getline(file, line)) {
    parseLine(line);
  }

  file.close();
  loaded = true;
  return true;
}

std::vector<std::vector<double>> GNSSDataReader::getData() const {
  return data;
}

void GNSSDataReader::parseLine(const std::string& line) {
  std::istringstream iss(line);
  std::vector<double> row;
  double value;
  char delimiter;

  while (iss >> value) {
    row.push_back(value);
  }

  data.push_back(row);
}