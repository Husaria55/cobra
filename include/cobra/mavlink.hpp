#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace cobra {

struct MavlinkMessage {
  std::uint8_t system_id;
  std::uint8_t component_id;
  std::uint32_t message_id;
  std::vector<std::uint8_t> payload;
};

class MavlinkParser {
 public:
  bool parse(std::uint8_t byte, MavlinkMessage& message);

 private:
  std::vector<std::uint8_t> frame_;
  std::size_t expected_size_ = 0;
};

class MavlinkConnection {
 public:
  MavlinkConnection(std::string endpoint, std::uint8_t system_id,
                    std::uint8_t component_id, unsigned int telemetry_rate_hz);
  ~MavlinkConnection();

  void connect();
  void run(const std::function<bool(const MavlinkMessage&)>& on_message,
           const std::function<bool()>& should_continue);
  bool requestTelemetry(const MavlinkMessage& heartbeat);
  void close();

 private:
  void sendHeartbeat();
  void sendMessageInterval(std::uint8_t target_system,
                           std::uint8_t target_component,
                           std::uint32_t message_id);
  void sendFrame(std::uint32_t message_id, const std::vector<std::uint8_t>& payload,
                 std::uint8_t crc_extra);

  std::string endpoint_;
  std::uint8_t system_id_;
  std::uint8_t component_id_;
  unsigned int telemetry_rate_hz_;
  int socket_ = -1;
  bool running_ = false;
  std::uint8_t sequence_ = 0;
  bool telemetry_requested_ = false;
};

std::string messageName(std::uint32_t message_id);
std::vector<std::string> messageValues(const MavlinkMessage& message);

}  // namespace cobra
