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
   * @brief 将 UDP 载荷字节解析为 CAN 帧。
   * @param payload 从以太网转 CAN 设备收到的原始字节。
   * @param channel 写入解析后帧中的逻辑通道 ID。
   * @param had_trailing_bytes 当载荷长度不是单条记录整数倍时置为 true。
   * @param trailing_bytes_count 载荷尾部被忽略的字节数。
   * @return 解析得到的 CAN 帧列表。
   */
  static std::vector<CanFrame> decodePayload(const std::vector<uint8_t> & payload, uint8_t channel,
    bool & had_trailing_bytes, size_t & trailing_bytes_count);

  /**
   * @brief Encode one CAN frame into fixed 13-byte Ethernet transport format.
   * @param frame CAN frame to encode.
   * @return Encoded 13-byte record.
   * @brief 将一帧 CAN 报文编码为固定 13 字节以太网传输格式。
   * @param frame 待编码的 CAN 帧。
   * @return 编码后的 13 字节记录。
   */
  static std::array<uint8_t, 13> encodeFrame(const CanFrame & frame);
};

}  // namespace chassis_driver
