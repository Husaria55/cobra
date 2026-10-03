#include "cobra/flight_log.hpp"

#include <ctime>
#include <iomanip>
#include <stdexcept>

namespace cobra {

std::string formatTime(std::chrono::system_clock::time_point time, const char* format) {
  const std::time_t raw_time = std::chrono::system_clock::to_time_t(time);
  std::tm local_time{};
  localtime_r(&raw_time, &local_time);
  std::ostringstream output;
  output << std::put_time(&local_time, format);
  return output.str();
}

std::string formatLogTimestamp(std::chrono::system_clock::time_point time) {
  return formatTime(time, "%d-%m-%Y %H:%M:%S");
}

FlightLog::FlightLog(const std::filesystem::path& flights_directory) {
  directory_ = flights_directory / formatTime(std::chrono::system_clock::now(), "%Y_%m_%d_%H_%M_%S");
  if (!std::filesystem::create_directories(directory_)) {
    throw std::runtime_error("flight directory already exists: " + directory_.string());
  }
  events_.open(directory_ / "events.log");
  telemetry_.open(directory_ / "telemetry.csv");
  if (!events_ || !telemetry_) throw std::runtime_error("cannot create flight logs");
  telemetry_ << "timestamp,message,system_id,component_id,values\n";
}

const std::filesystem::path& FlightLog::directory() const { return directory_; }

void FlightLog::event(const std::string& message) {
  events_ << formatLogTimestamp(std::chrono::system_clock::now()) << ' ' << message << '\n';
  events_.flush();
}

void FlightLog::telemetry(const std::string& message_name, std::uint8_t system_id,
                          std::uint8_t component_id,
                          const std::vector<std::string>& values) {
  telemetry_ << formatLogTimestamp(std::chrono::system_clock::now()) << ',' << message_name
             << ',' << static_cast<int>(system_id) << ','
             << static_cast<int>(component_id) << ",'";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) telemetry_ << "; ";
    telemetry_ << values[index];
  }
  telemetry_ << "'\n";
  telemetry_.flush();
}

}  // namespace cobra
