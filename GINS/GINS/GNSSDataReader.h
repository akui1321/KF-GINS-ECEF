#ifndef GNSS_DATA_READER_H
#define GNSS_DATA_READER_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

/**
 * @class GNSSDataReader
 * A class responsible for reading and parsing GNSS data from a file.
 */
class GNSSDataReader {
public:
  GNSSDataReader(const std::string& filename);

  bool loadData();

  std::vector<std::vector<double>> getData() const;

private:
  std::string filename;
  std::vector<std::vector<double>> data;
  bool loaded;

  void parseLine(const std::string& line);
};

#endif