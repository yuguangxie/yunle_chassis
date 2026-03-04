#pragma once

#include "chassis_driver/can_types.hpp"

#include <optional>
#include <unordered_map>

namespace chassis_driver
{

/**
 * @brief Static DBC signal encode/decode utility using in-code message definitions.
 */
class DbcProtocol
{
public:
  /** @brief Access frame-ID keyed message definitions. */
  static const std::unordered_map<uint32_t, MessageDefinition> & messageById();

  /** @brief Access message-name keyed message definitions. */
  static const std::unordered_map<std::string, MessageDefinition> & messageByName();

  /**
   * @brief Decode one physical signal value from a CAN frame.
   * @return Signal value when matched, std::nullopt otherwise.
   */
  static std::optional<double> decodeSignal(const CanFrame & frame, const std::string & signal_name);

  /**
   * @brief Encode one physical signal value into CAN frame payload.
   * @param frame In/out CAN frame.
   * @param signal_name Signal name defined in DBC map.
   * @param physical_value Engineering value to encode.
   * @param clamp Whether to clamp value to signal min/max range.
   * @return True when signal exists and is encoded.
   */
  static bool encodeSignal(
    CanFrame & frame, const std::string & signal_name, double physical_value, bool clamp = true);

private:
  /** @brief Extract Intel-endian bitfield value. */
  static uint64_t extractIntel(const std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length);

  /** @brief Extract Motorola-endian bitfield value. */
  static uint64_t extractMotorola(
    const std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length);

  /** @brief Insert Intel-endian bitfield value. */
  static void insertIntel(
    std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length, uint64_t raw_value);

  /** @brief Insert Motorola-endian bitfield value. */
  static void insertMotorola(
    std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length, uint64_t raw_value);

  /** @brief Sign-extend raw integer of given width to int64. */
  static int64_t signExtend(uint64_t raw_value, uint16_t bit_length);
};

}  // namespace chassis_driver
