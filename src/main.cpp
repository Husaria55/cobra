#include "cobra/config.hpp"
#include "cobra/flight_log.hpp"
#include "cobra/mavlink.hpp"

#include <atomic>
#include <csignal>
#include <iostream>
#include <stdexcept>

namespace {

std::atomic<bool> keep_running{true};

void stop(int) { keep_running = false; }

}  // namespace

int main(int argc, char** argv) {
  try {
    std::filesystem::path config_path = "config/sim.toml";
    if (argc == 3 && std::string(argv[1]) == "--config") config_path = argv[2];
    if (argc != 1 && argc != 3) throw std::runtime_error("usage: cobra [--config path]");

    const cobra::Config config = cobra::loadConfig(config_path);
    cobra::FlightLog log(config.flights_directory);
    cobra::MavlinkConnection connection(config.endpoint, config.system_id,
                                        config.component_id, config.telemetry_rate_hz);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    log.event("Cobra started with endpoint " + config.endpoint);
    std::cout << "Flight log: " << log.directory() << '\n';
    connection.connect();
    log.event("Connected to MAVLink endpoint");
    std::cout << "Connected to " << config.endpoint << ". Press Ctrl+C to finish.\n";
    connection.run(
        [&](const cobra::MavlinkMessage& message) {
          if (connection.requestTelemetry(message)) {
            log.event("Requested telemetry at " + std::to_string(config.telemetry_rate_hz) +
                      " Hz from system " + std::to_string(message.system_id));
          }
          const std::string name = cobra::messageName(message.message_id);
          if (!name.empty()) log.telemetry(name, message.system_id, message.component_id,
                                           cobra::messageValues(message));
          return true;
        },
        [] { return keep_running.load(); });
    log.event("Cobra stopped");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "cobra: " << error.what() << '\n';
    return 1;
  }
}
