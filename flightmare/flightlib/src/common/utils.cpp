#include "flightlib/common/utils.hpp"

namespace flightlib {

bool file_exists(const std::experimental::filesystem::path& p) {
  return (std::experimental::filesystem::exists(p));
}

}  // namespace flightlib