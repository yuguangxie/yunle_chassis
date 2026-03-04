#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace chassis_driver
{

enum class ByteOrder
{
  Intel,
  Motorola,
};

enum class ValueType
{
  Unsigned,
  Signed,
};

struct SignalDefinition
{
  std::string name;
  uint16_t start_bit;
  uint16_t bit_length;
  ByteOrder byte_order;
  ValueType value_type;
  double factor;
  double offset;
  double min_value;
  double max_value;
  std::string unit;
};

struct MessageDefinition
{
  std::string name;
  uint32_t frame_id;
  bool is_extended;
  uint8_t dlc;
  std::string sender;
  std::vector<SignalDefinition> signals;
};

struct CanFrame
{
  uint32_t can_id{0};
  bool is_extended{false};
  bool is_remote{false};
  uint8_t dlc{0};
  std::array<uint8_t, 8> data{};
  uint8_t channel{0};
};

}  // namespace chassis_driver
