#pragma once

#include "chassis_driver/can_types.hpp"

#include <optional>
#include <unordered_map>

namespace chassis_driver
{

/**
 * @brief Static DBC signal encode/decode utility using in-code message definitions.
 * @brief 使用代码内置报文定义的静态 DBC 信号编解码工具。
 */
class DbcProtocol
{
public:
  /** @brief Access frame-ID keyed message definitions. */
  /** @brief 获取以 CAN ID 为 key 的报文定义。 */
  static const std::unordered_map<uint32_t, MessageDefinition> & messageById();

  /** @brief Access message-name keyed message definitions. */
  /** @brief 获取以报文名称为 key 的报文定义。 */
  static const std::unordered_map<std::string, MessageDefinition> & messageByName();

  /**
   * @brief Decode one physical signal value from a CAN frame.
   * @return Signal value when matched, std::nullopt otherwise.
   * @brief 从 CAN 帧中解码一个物理信号值。
   * @return 匹配到信号时返回信号值，否则返回 std::nullopt。
   */
  static std::optional<double> decodeSignal(const CanFrame & frame, const std::string & signal_name);

  /**
   * @brief Encode one physical signal value into CAN frame payload.
   * @param frame In/out CAN frame.
   * @param signal_name Signal name defined in DBC map.
   * @param physical_value Engineering value to encode.
   * @param clamp Whether to clamp value to signal min/max range.
   * @return True when signal exists and is encoded.
   * @brief 将一个物理信号值编码到 CAN 帧 payload 中。
   * @param frame 输入/输出 CAN 帧。
   * @param signal_name DBC 映射中定义的信号名。
   * @param physical_value 待编码的工程值。
   * @param clamp 是否将数值限制在信号最小/最大范围内。
   * @return 信号存在且编码成功时返回 true。
   */
  static bool encodeSignal(
    CanFrame & frame, const std::string & signal_name, double physical_value, bool clamp = true);

private:
  /** @brief Extract Intel-endian bitfield value. */
  /** @brief 提取 Intel 小端 bitfield 数值。 */
  static uint64_t extractIntel(const std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length);

  /** @brief Extract Motorola-endian bitfield value. */
  /** @brief 提取 Motorola 大端 bitfield 数值。 */
  static uint64_t extractMotorola(
    const std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length);

  /** @brief Insert Intel-endian bitfield value. */
  /** @brief 写入 Intel 小端 bitfield 数值。 */
  static void insertIntel(
    std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length, uint64_t raw_value);

  /** @brief Insert Motorola-endian bitfield value. */
  /** @brief 写入 Motorola 大端 bitfield 数值。 */
  static void insertMotorola(
    std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length, uint64_t raw_value);

  /** @brief Sign-extend raw integer of given width to int64. */
  /** @brief 将指定位宽的 raw 整数符号扩展为 int64。 */
  static int64_t signExtend(uint64_t raw_value, uint16_t bit_length);
};

}  // namespace chassis_driver
