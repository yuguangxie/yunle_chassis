#pragma once

#include "chassis_driver/can_types.hpp"

#include <optional>
#include <unordered_map>

namespace chassis_driver
{

class DbcProtocol
{
public:
  static const std::unordered_map<uint32_t, MessageDefinition> & messageById();
  static const std::unordered_map<std::string, MessageDefinition> & messageByName();

  static std::optional<double> decodeSignal(const CanFrame & frame, const std::string & signal_name);
  static bool encodeSignal(
    CanFrame & frame, const std::string & signal_name, double physical_value, bool clamp = true);

private:
  static uint64_t extractIntel(const std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length);
  static uint64_t extractMotorola(
    const std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length);
  static void insertIntel(
    std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length, uint64_t raw_value);
  static void insertMotorola(
    std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length, uint64_t raw_value);
  static int64_t signExtend(uint64_t raw_value, uint16_t bit_length);
};

}  // namespace chassis_driver
