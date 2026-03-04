#pragma once

#include "chassis_driver/can_types.hpp"

#include <cstdint>
#include <vector>

namespace chassis_driver
{

class CanEthernetCodec
{
public:
  /**
   * @brief Decode UDP payload bytes into CAN frames.
   * @param payload Raw bytes received from CAN-over-Ethernet device.
   * @param channel Logical channel ID to stamp into decoded frames.
   * @param had_trailing_bytes Set true when payload length is not a multiple of one record.
   * @param trailing_bytes_count Number of bytes ignored at payload tail.
   * @return Decoded CAN frames.
   */
  static std::vector<CanFrame> decodePayload(const std::vector<uint8_t> & payload, uint8_t channel,
    bool & had_trailing_bytes, size_t & trailing_bytes_count);

  /**
   * @brief Encode one CAN frame into fixed 13-byte Ethernet transport format.
   * @param frame CAN frame to encode.
   * @return Encoded 13-byte record.
   */
  static std::array<uint8_t, 13> encodeFrame(const CanFrame & frame);
};

}  // namespace chassis_driver
