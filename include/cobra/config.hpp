#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace cobra {

struct Config {
  std::string endpoint;
  std::uint8_t system_id;
  std::uint8_t component_id;
  unsigned int telemetry_rate_hz;
  std::filesystem::path flights_directory;
};

Config loadConfig(const std::filesystem::path& path);

}  // namespace cobra
