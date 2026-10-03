#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace cobra {

class FlightLog {
 public:
  explicit FlightLog(const std::filesystem::path& flights_directory);

  const std::filesystem::path& directory() const;
  void event(const std::string& message);
  void telemetry(const std::string& message_name, std::uint8_t system_id,
                 std::uint8_t component_id,
                 const std::vector<std::string>& values);

 private:
  std::filesystem::path directory_;
  std::ofstream events_;
  std::ofstream telemetry_;
};

std::string formatLogTimestamp(std::chrono::system_clock::time_point time);

}  // namespace cobra
