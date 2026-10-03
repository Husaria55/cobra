#include "cobra/mavlink.hpp"

#include <array>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <iomanip>
#include <netdb.h>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace cobra {
namespace {

constexpr std::uint8_t kMavlinkV1Start = 0xfe;
constexpr std::uint8_t kMavlinkV2Start = 0xfd;

std::uint16_t crcAccumulate(std::uint8_t value, std::uint16_t crc) {
  std::uint8_t temporary = value ^ static_cast<std::uint8_t>(crc);
  temporary ^= temporary << 4;
  return (crc >> 8) ^ (static_cast<std::uint16_t>(temporary) << 8) ^
         (static_cast<std::uint16_t>(temporary) << 3) ^ (temporary >> 4);
}

std::uint16_t crc(const std::vector<std::uint8_t>& bytes, std::size_t start,
                  std::size_t count, std::uint8_t extra) {
  std::uint16_t value = 0xffff;
  for (std::size_t index = start; index < start + count; ++index) {
    value = crcAccumulate(bytes[index], value);
  }
  return crcAccumulate(extra, value);
}

std::uint8_t crcExtra(std::uint32_t message_id) {
  switch (message_id) {
    case 0: return 50;
    case 1: return 124;
    case 24: return 24;
    case 30: return 39;
    case 33: return 104;
    case 74: return 20;
    case 147: return 154;
    default: return 0;
  }
}

template <typename T>
T valueAt(const std::vector<std::uint8_t>& payload, std::size_t offset) {
  if (offset + sizeof(T) > payload.size()) {
    throw std::runtime_error("short MAVLink payload");
  }
  T value{};
  std::memcpy(&value, payload.data() + offset, sizeof(value));
  return value;
}

std::string number(double value, int precision = 3) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

std::pair<std::string, std::string> parseTcpEndpoint(const std::string& endpoint) {
  constexpr const char* prefix = "tcp:";
  if (endpoint.rfind(prefix, 0) != 0) {
    throw std::runtime_error("only tcp:host:port endpoints are supported");
  }
  const std::string address = endpoint.substr(std::strlen(prefix));
  const std::size_t colon = address.rfind(':');
  if (colon == std::string::npos || colon == 0 || colon == address.size() - 1) {
    throw std::runtime_error("endpoint must have the form tcp:host:port");
  }
  return {address.substr(0, colon), address.substr(colon + 1)};
}

}  // namespace

bool MavlinkParser::parse(std::uint8_t byte, MavlinkMessage& message) {
  if (frame_.empty()) {
    if (byte == kMavlinkV1Start || byte == kMavlinkV2Start) frame_.push_back(byte);
    return false;
  }

  frame_.push_back(byte);
  if (frame_.size() == 2 && frame_[0] == kMavlinkV1Start) {
    expected_size_ = 8 + frame_[1];
  }
  if (frame_.size() == 3 && frame_[0] == kMavlinkV2Start) {
    expected_size_ = 12 + frame_[1] + ((frame_[2] & 0x01) ? 13 : 0);
  }
  if (expected_size_ == 0 || frame_.size() < expected_size_) return false;

  const bool v2 = frame_[0] == kMavlinkV2Start;
  const std::size_t header_size = v2 ? 10 : 6;
  const std::size_t payload_size = frame_[1];
  const std::uint32_t message_id = v2
      ? static_cast<std::uint32_t>(frame_[7]) |
            (static_cast<std::uint32_t>(frame_[8]) << 8) |
            (static_cast<std::uint32_t>(frame_[9]) << 16)
      : frame_[5];
  const std::uint8_t extra = crcExtra(message_id);
  const std::size_t checksum_offset = header_size + payload_size;
  const std::uint16_t received_checksum = frame_[checksum_offset] |
      (static_cast<std::uint16_t>(frame_[checksum_offset + 1]) << 8);
  const std::uint16_t calculated_checksum = crc(frame_, 1, header_size - 1 + payload_size, extra);
  const bool valid = extra != 0 && received_checksum == calculated_checksum;
  if (valid) {
    message.system_id = frame_[v2 ? 5 : 3];
    message.component_id = frame_[v2 ? 6 : 4];
    message.message_id = message_id;
    message.payload.assign(frame_.begin() + header_size,
                           frame_.begin() + header_size + payload_size);
  }
  frame_.clear();
  expected_size_ = 0;
  return valid;
}

MavlinkConnection::MavlinkConnection(std::string endpoint, std::uint8_t system_id,
                                     std::uint8_t component_id, unsigned int telemetry_rate_hz)
    : endpoint_(std::move(endpoint)), system_id_(system_id), component_id_(component_id),
      telemetry_rate_hz_(telemetry_rate_hz) {}

MavlinkConnection::~MavlinkConnection() { close(); }

void MavlinkConnection::connect() {
  const auto [host, port] = parseTcpEndpoint(endpoint_);
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* addresses = nullptr;
  const int status = getaddrinfo(host.c_str(), port.c_str(), &hints, &addresses);
  if (status != 0) throw std::runtime_error("cannot resolve MAVLink endpoint: " + std::string(gai_strerror(status)));

  for (addrinfo* address = addresses; address != nullptr; address = address->ai_next) {
    const int candidate = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (candidate < 0) continue;
    if (::connect(candidate, address->ai_addr, address->ai_addrlen) == 0) {
      socket_ = candidate;
      break;
    }
    ::close(candidate);
  }
  freeaddrinfo(addresses);
  if (socket_ < 0) throw std::runtime_error("cannot connect to MAVLink endpoint " + endpoint_);
  running_ = true;
}

void MavlinkConnection::run(const std::function<bool(const MavlinkMessage&)>& on_message,
                            const std::function<bool()>& should_continue) {
  MavlinkParser parser;
  auto next_heartbeat = std::chrono::steady_clock::now();
  std::array<std::uint8_t, 512> bytes{};
  while (running_ && should_continue()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_heartbeat) {
      sendHeartbeat();
      next_heartbeat = now + std::chrono::seconds(1);
    }
    pollfd descriptor{socket_, POLLIN, 0};
    const int ready = poll(&descriptor, 1, 200);
    if (ready < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error("MAVLink socket poll failed");
    }
    if (ready == 0) continue;
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      throw std::runtime_error("MAVLink connection closed by endpoint");
    }
    const ssize_t count = recv(socket_, bytes.data(), bytes.size(), 0);
    if (count <= 0) throw std::runtime_error("MAVLink connection closed by endpoint");
    for (ssize_t index = 0; index < count; ++index) {
      MavlinkMessage message{};
      if (parser.parse(bytes[static_cast<std::size_t>(index)], message) && !on_message(message)) {
        running_ = false;
        return;
      }
    }
  }
}

void MavlinkConnection::close() {
  running_ = false;
  if (socket_ >= 0) {
    ::shutdown(socket_, SHUT_RDWR);
    ::close(socket_);
    socket_ = -1;
  }
}

void MavlinkConnection::sendHeartbeat() {
  sendFrame(0, {0, 0, 0, 0, 18, 8, 0, 4, 3}, 50);
}

bool MavlinkConnection::requestTelemetry(const MavlinkMessage& heartbeat) {
  if (telemetry_requested_ || heartbeat.message_id != 0 || heartbeat.payload.size() < 6 ||
      heartbeat.payload[5] == 8) {
    return false;
  }
  for (const std::uint32_t message_id : {1U, 24U, 30U, 33U, 74U, 147U}) {
    sendMessageInterval(heartbeat.system_id, heartbeat.component_id, message_id);
  }
  telemetry_requested_ = true;
  return true;
}

void MavlinkConnection::sendMessageInterval(std::uint8_t target_system,
                                            std::uint8_t target_component,
                                            std::uint32_t message_id) {
  std::vector<std::uint8_t> payload(33, 0);
  const float requested_message = static_cast<float>(message_id);
  const float interval_us = 1'000'000.0F / static_cast<float>(telemetry_rate_hz_);
  std::memcpy(payload.data(), &requested_message, sizeof(requested_message));
  std::memcpy(payload.data() + 4, &interval_us, sizeof(interval_us));
  payload[28] = 0xff;
  payload[29] = 0x01;
  payload[30] = target_system;
  payload[31] = target_component;
  sendFrame(76, payload, 152);
}

void MavlinkConnection::sendFrame(std::uint32_t message_id,
                                  const std::vector<std::uint8_t>& payload,
                                  std::uint8_t crc_extra) {
  std::vector<std::uint8_t> frame{0xfd, static_cast<std::uint8_t>(payload.size()), 0, 0,
                                  sequence_++, system_id_, component_id_,
                                  static_cast<std::uint8_t>(message_id),
                                  static_cast<std::uint8_t>(message_id >> 8),
                                  static_cast<std::uint8_t>(message_id >> 16)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  const std::uint16_t checksum = crc(frame, 1, frame.size() - 1, crc_extra);
  frame.push_back(static_cast<std::uint8_t>(checksum));
  frame.push_back(static_cast<std::uint8_t>(checksum >> 8));
  std::size_t sent = 0;
  while (sent < frame.size()) {
    const ssize_t result = send(socket_, frame.data() + sent, frame.size() - sent, MSG_NOSIGNAL);
    if (result <= 0) throw std::runtime_error("cannot send MAVLink message");
    sent += static_cast<std::size_t>(result);
  }
}

std::string messageName(std::uint32_t message_id) {
  switch (message_id) {
    case 0: return "HEARTBEAT";
    case 1: return "SYS_STATUS";
    case 24: return "GPS_RAW_INT";
    case 30: return "ATTITUDE";
    case 33: return "GLOBAL_POSITION_INT";
    case 74: return "VFR_HUD";
    case 147: return "BATTERY_STATUS";
    default: return "";
  }
}

std::vector<std::string> messageValues(const MavlinkMessage& message) {
  try {
    switch (message.message_id) {
      case 0:
        return {"type=" + std::to_string(valueAt<std::uint8_t>(message.payload, 4)),
                "autopilot=" + std::to_string(valueAt<std::uint8_t>(message.payload, 5)),
                "base_mode=" + std::to_string(valueAt<std::uint8_t>(message.payload, 6)),
                "system_status=" + std::to_string(valueAt<std::uint8_t>(message.payload, 7))};
      case 1:
        return {"load=" + number(valueAt<std::uint16_t>(message.payload, 12) / 10.0, 1) + "%",
                "voltage=" + number(valueAt<std::uint16_t>(message.payload, 14) / 1000.0) + "V",
                "current=" + number(valueAt<std::int16_t>(message.payload, 16) / 100.0) + "A",
                "remaining=" + std::to_string(valueAt<std::int8_t>(message.payload, 18)) + "%"};
      case 24:
        return {"fix_type=" + std::to_string(valueAt<std::uint8_t>(message.payload, 28)),
                "satellites=" + std::to_string(valueAt<std::uint8_t>(message.payload, 29)),
                "latitude=" + number(valueAt<std::int32_t>(message.payload, 8) / 1e7, 7),
                "longitude=" + number(valueAt<std::int32_t>(message.payload, 12) / 1e7, 7),
                "altitude=" + number(valueAt<std::int32_t>(message.payload, 16) / 1000.0) + "m"};
      case 30:
        return {"roll=" + number(valueAt<float>(message.payload, 4)) + "rad",
                "pitch=" + number(valueAt<float>(message.payload, 8)) + "rad",
                "yaw=" + number(valueAt<float>(message.payload, 12)) + "rad"};
      case 33:
        return {"latitude=" + number(valueAt<std::int32_t>(message.payload, 4) / 1e7, 7),
                "longitude=" + number(valueAt<std::int32_t>(message.payload, 8) / 1e7, 7),
                "altitude=" + number(valueAt<std::int32_t>(message.payload, 12) / 1000.0) + "m",
                "relative_altitude=" + number(valueAt<std::int32_t>(message.payload, 16) / 1000.0) + "m"};
      case 74:
        return {"airspeed=" + number(valueAt<float>(message.payload, 0)) + "m/s",
                "groundspeed=" + number(valueAt<float>(message.payload, 4)) + "m/s",
                "heading=" + std::to_string(valueAt<std::int16_t>(message.payload, 8)) + "deg",
                "altitude=" + number(valueAt<float>(message.payload, 12)) + "m"};
      case 147:
        return {"current=" + number(valueAt<std::int16_t>(message.payload, 30) / 100.0) + "A",
                "remaining=" + std::to_string(valueAt<std::int8_t>(message.payload, 35)) + "%"};
      default: return {};
    }
  } catch (const std::runtime_error&) {
    return {"malformed payload"};
  }
}

}  // namespace cobra
