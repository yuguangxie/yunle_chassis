#include "chassis_driver/can_ethernet_codec.hpp"

namespace chassis_driver
{

/** Decode fixed-size CAN-over-Ethernet records from payload bytes. */
std::vector<CanFrame> CanEthernetCodec::decodePayload(const std::vector<uint8_t> & payload, uint8_t channel,
  bool & had_trailing_bytes, size_t & trailing_bytes_count)
{
  std::vector<CanFrame> frames;
  had_trailing_bytes = false;
  trailing_bytes_count = payload.size() % 13U;
  if (trailing_bytes_count != 0U) {
    had_trailing_bytes = true;
  }

  const size_t records = payload.size() / 13U;
  frames.reserve(records);
  for (size_t i = 0; i < records; ++i) {
    const size_t base = i * 13U;
    const uint8_t info = payload[base];
    CanFrame frame;
    frame.is_extended = (info & 0x80U) != 0U;
    frame.is_remote = (info & 0x40U) != 0U;
    frame.dlc = static_cast<uint8_t>(info & 0x0FU);
    if (frame.dlc > 8U) {
      frame.dlc = 8U;
    }

    uint32_t id = (static_cast<uint32_t>(payload[base + 1]) << 24U) |
      (static_cast<uint32_t>(payload[base + 2]) << 16U) |
      (static_cast<uint32_t>(payload[base + 3]) << 8U) |
      static_cast<uint32_t>(payload[base + 4]);
    frame.can_id = frame.is_extended ? (id & 0x1FFFFFFFU) : (id & 0x7FFU);
    for (size_t b = 0; b < 8; ++b) {
      frame.data[b] = payload[base + 5U + b];
    }
    frame.channel = channel;
    frames.push_back(frame);
  }
  return frames;
}

/** Encode CAN frame into one 13-byte transport record. */
std::array<uint8_t, 13> CanEthernetCodec::encodeFrame(const CanFrame & frame)
{
  std::array<uint8_t, 13> out{};
  const uint8_t dlc = frame.dlc > 8U ? 8U : frame.dlc;
  out[0] = static_cast<uint8_t>((frame.is_extended ? 0x80U : 0x00U) | 0x20U | (dlc & 0x0FU));
  out[1] = static_cast<uint8_t>((frame.can_id >> 24U) & 0xFFU);
  out[2] = static_cast<uint8_t>((frame.can_id >> 16U) & 0xFFU);
  out[3] = static_cast<uint8_t>((frame.can_id >> 8U) & 0xFFU);
  out[4] = static_cast<uint8_t>(frame.can_id & 0xFFU);
  for (size_t i = 0; i < 8; ++i) {
    out[5U + i] = frame.data[i];
  }
  return out;
}

}  // namespace chassis_driver
