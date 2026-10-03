#include "cobra/config.hpp"

#include <cctype>
#include <fstream>
#include <stdexcept>

namespace cobra {
namespace {

std::string trim(std::string text) {
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::string quotedValue(const std::string& value, const std::string& key) {
  if (value.size() < 2 || value.front() != '\"' || value.back() != '\"') {
    throw std::runtime_error(key + " must be a quoted TOML string");
  }
  return value.substr(1, value.size() - 2);
}

std::uint8_t byteValue(const std::string& value, const std::string& key) {
  const int number = std::stoi(value);
  if (number < 0 || number > 255) {
    throw std::runtime_error(key + " must be between 0 and 255");
  }
  return static_cast<std::uint8_t>(number);
}

unsigned int rateValue(const std::string& value) {
  const int number = std::stoi(value);
  if (number < 1 || number > 50) {
    throw std::runtime_error("mavlink.telemetry_rate_hz must be between 1 and 50");
  }
  return static_cast<unsigned int>(number);
}

}  // namespace

Config loadConfig(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open configuration: " + path.string());

  Config config{"", 255, 191, 2, "cobra_flights"};
  std::string section;
  std::string line;
  while (std::getline(file, line)) {
    line = trim(line.substr(0, line.find('#')));
    if (line.empty()) continue;
    if (line.front() == '[' && line.back() == ']') {
      section = line.substr(1, line.size() - 2);
      continue;
    }
    const auto equals = line.find('=');
    if (equals == std::string::npos) {
      throw std::runtime_error("invalid TOML line: " + line);
    }
    const std::string key = trim(line.substr(0, equals));
    const std::string value = trim(line.substr(equals + 1));
    if (section == "mavlink" && key == "endpoint") {
      config.endpoint = quotedValue(value, "mavlink.endpoint");
    } else if (section == "mavlink" && key == "system_id") {
      config.system_id = byteValue(value, "mavlink.system_id");
    } else if (section == "mavlink" && key == "component_id") {
      config.component_id = byteValue(value, "mavlink.component_id");
    } else if (section == "mavlink" && key == "telemetry_rate_hz") {
      config.telemetry_rate_hz = rateValue(value);
    } else if (section == "logging" && key == "flights_directory") {
      config.flights_directory = quotedValue(value, "logging.flights_directory");
    }
  }
  if (config.endpoint.empty()) throw std::runtime_error("mavlink.endpoint is required");
  return config;
}

}  // namespace cobra
