#pragma once

#include <experimental/filesystem>

namespace flightlib {

bool file_exists(const std::experimental::filesystem::path& p);

}  // namespace flightlib