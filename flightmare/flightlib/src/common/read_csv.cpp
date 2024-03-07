#include "flightlib/common/read_csv.hpp"

namespace flightlib {

using namespace std;

bool readCsv(std::string path, Scalar &fill) { 
  fill = read(path)(0, 0);
  return true;
}

bool readCsv(std::string path, Ref<Matrix<Dynamic, Dynamic>> fill) {
  fill = read(path);
  return true;
}

bool readCsv(std::string path,
             Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> &fill) {
  Matrix<Dynamic, Dynamic> content_m = read(path);

  if (content_m.size() !=
      fill.dimension(0) * fill.dimension(1) * fill.dimension(2)) {
    std::cout << "read_csv: dimensions don't match\n";
    return false;
  }

  for (int i = 0; i < fill.dimension(0); i++) {
    for (int j = 0; j < fill.dimension(1); j++) {
      for (int k = 0; k < fill.dimension(2); k++) {
        fill(i, j, k) = content_m(i, j + k * fill.dimension(1));
      }
    }
  }
  return true;
}

bool readCsv(std::string path,
             Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> &fill) {
  Matrix<Dynamic, Dynamic> content_m = read(path);

  if (content_m.size() !=
      fill.dimension(0) * fill.dimension(1) * fill.dimension(2)) {
    std::cout << "read_csv: dimensions don't match\n";
    return false;
  }

  for (int i = 0; i < fill.dimension(0); i++) {
    for (int j = 0; j < fill.dimension(1); j++) {
      for (int k = 0; k < fill.dimension(2); k++) {
        fill(i, j, k) = content_m(i, j + k * fill.dimension(1));
      }
    }
  }
  return true;
}

bool readCsv(std::string path,
             Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> &fill) {
  Matrix<Dynamic, Dynamic> content_m = read(path);

  if (content_m.size() !=
      fill.dimension(0) * fill.dimension(1) * fill.dimension(2)) {
    std::cout << "read_csv: dimensions don't match\n";
    return false;
  }

  for (int i = 0; i < fill.dimension(0); i++) {
    for (int j = 0; j < fill.dimension(1); j++) {
      for (int k = 0; k < fill.dimension(2); k++) {
        fill(i, j, k) = content_m(i, j + k * fill.dimension(1));
      }
    }
  }
  return true;
}

Matrix<Dynamic, Dynamic> read(std::string path) {
  std::vector<std::vector<string>> content;
  std::vector<std::string> row;
  std::string line, word;

  fstream file(path, ios::in);
  if (file.is_open()) {
    while (getline(file, line)) {
      row.clear();

      stringstream str(line);

      while (getline(str, word, ',')) row.push_back(word);
      content.push_back(row);
    }
  } else
    cout << "read_csv: could not open the file\n";

  MatrixXd content_m(content.size(), content[0].size());
  
  for (int i = 0; (unsigned) i < content.size(); i++) {
    for (int j = 0; (unsigned) j < content[i].size(); j++) {
      content_m(i, j) = std::stof(content[i][j]);
    }
  }
  return content_m;
}

}  // namespace flightlib