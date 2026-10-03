#include "cobra/mavlink.hpp"

#include <cstdint>
#include <iostream>
#include <vector>

namespace {

std::uint16_t accumulate(std::uint8_t value, std::uint16_t crc) {
  std::uint8_t temporary = value ^ static_cast<std::uint8_t>(crc);
  temporary ^= temporary << 4;
  return (crc >> 8) ^ (static_cast<std::uint16_t>(temporary) << 8) ^
         (static_cast<std::uint16_t>(temporary) << 3) ^ (temporary >> 4);
}

std::vector<std::uint8_t> heartbeat() {
  std::vector<std::uint8_t> frame{0xfd, 9, 0, 0, 7, 1, 1, 0, 0, 0,
                                  0, 0, 0, 0, 2, 3, 0, 4, 3};
  std::uint16_t crc = 0xffff;
  for (std::size_t index = 1; index < frame.size(); ++index) crc = accumulate(frame[index], crc);
  crc = accumulate(50, crc);
  frame.push_back(static_cast<std::uint8_t>(crc));
  frame.push_back(static_cast<std::uint8_t>(crc >> 8));
  return frame;
}

}  // namespace

int main() {
  cobra::MavlinkParser parser;
  cobra::MavlinkMessage message{};
  bool complete = false;
  for (const std::uint8_t byte : heartbeat()) complete = parser.parse(byte, message) || complete;
  if (!complete || message.message_id != 0 || message.system_id != 1 ||
      message.component_id != 1 || message.payload.size() != 9) {
    std::cerr << "valid heartbeat was not decoded\n";
    return 1;
  }

  auto invalid = heartbeat();
  invalid.back() ^= 0xff;
  for (const std::uint8_t byte : invalid) {
    if (parser.parse(byte, message)) {
      std::cerr << "invalid heartbeat was accepted\n";
      return 1;
    }
  }
  return 0;
}
