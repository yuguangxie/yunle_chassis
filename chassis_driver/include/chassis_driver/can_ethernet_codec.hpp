#pragma once

#include "chassis_driver/can_types.hpp"

#include <cstdint>
#include <vector>

namespace chassis_driver
{

class CanEthernetCodec
{
public:
  static std::vector<CanFrame> decodePayload(const std::vector<uint8_t> & payload, uint8_t channel,
    bool & had_trailing_bytes, size_t & trailing_bytes_count);

  static std::array<uint8_t, 13> encodeFrame(const CanFrame & frame);
};

}  // namespace chassis_driver
