#include "chassis_driver/dbc_protocol.hpp"

#include <cmath>
#include <limits>

namespace chassis_driver
{

namespace
{

const std::unordered_map<uint32_t, MessageDefinition> kMessageById = {
  {256U, {"BMS_Status", 256U, false, 8U, "BMS", {
      {"BMS_Voltage", 0, 16, ByteOrder::Intel, ValueType::Unsigned, 0.1, 0.0, 0.0, 6553.5, "V"},
      {"BMS_Current", 16, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "A"},
      {"BMS_SOC", 32, 16, ByteOrder::Intel, ValueType::Unsigned, 0.1, 0.0, 0.0, 100.0, "%"},
    }}},
  {119U, {"VCU_Warning_Level", 119U, false, 8U, "VCU", {
      {"BMS_Charge_Current_Warning", 0, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"BMS_Discharge_Current_Warning", 3, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"BMS_SOC_Warning", 6, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"BMS_Temperature_Warning", 9, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"MCU_Current_Warning", 12, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"MCU_Disconnect_Warning", 15, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"MCU_Motor_Warning", 18, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"MCU_Speed_Warning", 21, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"MCU_Temperature_Warning", 24, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"MCU_Voltage_Warning", 27, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"Steering_Disconnect_Warning", 30, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"Steering_Lock_Warning", 33, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
      {"Steering_Uncontrollable_Warning", 36, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 7.0, ""},
    }}},
  {360U, {"VCU_Wheel_Speed_Feedback", 360U, false, 8U, "VCU", {
      {"Wheel_Speed_Front_Left_RPM", 0, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "rpm"},
      {"Wheel_Speed_Front_Right_RPM", 16, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "rpm"},
      {"Wheel_Speed_Rear_Left_RPM", 32, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "rpm"},
      {"Wheel_Speed_Rear_Right_RPM", 48, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "rpm"},
    }}},
  {81U, {"VCU_CCU_Status", 81U, false, 8U, "VCU", {
      {"CCU_Shift_Level_Status", 0, 2, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 3.0, ""},
      {"CCU_Parking_Status", 2, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"CCU_Ignition_Status", 3, 2, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 3.0, ""},
      {"CCU_Drive_Mode_Shift_Button", 5, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"Steering_Wheel_Direction", 7, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"CCU_Steering_Wheel_Angle", 8, 12, ByteOrder::Intel, ValueType::Unsigned, 0.1, 0.0, 0.0, 120.0, ""},
      {"CCU_Vehicle_Speed", 20, 9, ByteOrder::Intel, ValueType::Unsigned, 0.1, 0.0, 0.0, 51.1, "km/h"},
      {"CCU_Drive_Mode", 29, 3, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 3.0, ""},
      {"Remote_Brake_Request_Status", 32, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"Emergency_Brake_Request_Status", 33, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"SCU_Brake_Signal_Status", 34, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"Left_Turn_Light_Status", 56, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"Right_Turn_Light_Status", 57, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"Position_Light_Status", 59, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"Low_Beam_Status", 60, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
    }}},
  {225U, {"SAS_Angle_Feedback", 225U, false, 8U, "SAS_SW", {
      {"SAS_Front_Angle", 0, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "deg"},
      {"SAS_Rear_Angle", 24, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "deg"},
    }}},
  {2033U, {"SCU_Target_Speed_Feedback", 2033U, false, 8U, "SCU", {
      {"Hardware_Target_Speed", 0, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "km/h"},
      {"SCU_Target_Speed_Feedback", 16, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "km/h"},
      {"Vehicle_Target_Speed", 32, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "km/h"},
      {"Vehicle_Target_Speed_RPM", 48, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "rpm"},
    }}},
  {291U, {"SCU_Torque_Command", 291U, false, 8U, "SCU", {
      {"Torque_Command_Front_Left", 0, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "Nm"},
      {"Torque_Command_Front_Right", 16, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "Nm"},
      {"Torque_Command_Rear_Left", 32, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "Nm"},
      {"Torque_Command_Rear_Right", 48, 16, ByteOrder::Intel, ValueType::Signed, 0.1, 0.0, -3276.8, 3276.7, "Nm"},
    }}},
  {294U, {"SCU_Chassis_Command", 294U, false, 8U, "SCU", {
      {"VCU_Target_Steering_Angle_Speed", 0, 16, ByteOrder::Intel, ValueType::Signed, 1.0, 0.0, 126.0, 525.0, "deg/s"},
      {"Brake_Force_Front_Left", 16, 8, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 100.0, "%"},
      {"Brake_Force_Front_Right", 24, 8, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 100.0, "%"},
      {"Brake_Force_Rear_Left", 32, 8, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 100.0, "%"},
      {"Brake_Force_Rear_Right", 40, 8, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 100.0, "%"},
    }}},
  {289U, {"SCU_Control_Command", 289U, false, 8U, "SCU", {
      {"SCU_Shift_Level_Request", 0, 2, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 3.0, ""},
      {"SCU_Drive_Mode_Request", 6, 2, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 3.0, ""},
      {"SCU_Steering_Angle_JD01_Front", 8, 8, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 255.0, ""},
      {"SCU_Steering_Angle_JD01_Rear", 16, 8, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 255.0, ""},
      {"SCU_Target_Speed", 24, 9, ByteOrder::Intel, ValueType::Unsigned, 0.1, 0.0, 0.0, 51.1, "km/h"},
      {"SCU_Brake_Enable", 33, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"GW_Left_Turn_Light_Request", 40, 2, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 3.0, ""},
      {"GW_Right_Turn_Light_Request", 42, 2, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 3.0, ""},
      {"GW_Position_Light_Request", 46, 2, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 3.0, ""},
      {"GW_Low_Beam_Request", 48, 2, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 3.0, ""},
      {"SCU_Torque_Or_Speed_Mode", 58, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"Steering_Angle_Speed_Valid", 60, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
      {"Brake_Force_Command_Valid", 61, 1, ByteOrder::Intel, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
    }}},
  {2542813185U, {"BMS_Realtime_Status", 2542813185U, true, 8U, "BMS", {
      {"BMS_Total_Battery_Voltage", 0, 16, ByteOrder::Intel, ValueType::Unsigned, 0.1, 0.0, 0.0, 6553.5, "V"},
      {"BMS_Pack_Voltage", 16, 16, ByteOrder::Intel, ValueType::Unsigned, 0.1, 0.0, 0.0, 6553.5, "V"},
      {"BMS_Pack_Current", 32, 16, ByteOrder::Intel, ValueType::Unsigned, 0.1, -3000.0, -3000.0, 3553.5, "A"},
      {"BMS_Realtime_SOC", 48, 16, ByteOrder::Intel, ValueType::Unsigned, 0.1, 0.0, 0.0, 100.0, "%"},
    }}},
};

std::unordered_map<std::string, MessageDefinition> createByName()
{
  std::unordered_map<std::string, MessageDefinition> by_name;
  for (const auto & kv : kMessageById) {
    by_name.emplace(kv.second.name, kv.second);
  }
  return by_name;
}

const std::unordered_map<std::string, MessageDefinition> kMessageByName = createByName();

}  // namespace

const std::unordered_map<uint32_t, MessageDefinition> & DbcProtocol::messageById() { return kMessageById; }
const std::unordered_map<std::string, MessageDefinition> & DbcProtocol::messageByName() { return kMessageByName; }

std::optional<double> DbcProtocol::decodeSignal(const CanFrame & frame, const std::string & signal_name)
{
  const auto msg_it = kMessageById.find(frame.can_id);
  if (msg_it == kMessageById.end()) { return std::nullopt; }
  for (const auto & sig : msg_it->second.signals) {
    if (sig.name == signal_name) {
      const uint64_t raw = sig.byte_order == ByteOrder::Intel ?
        extractIntel(frame.data, sig.start_bit, sig.bit_length) :
        extractMotorola(frame.data, sig.start_bit, sig.bit_length);
      const double signed_raw = (sig.value_type == ValueType::Signed) ?
        static_cast<double>(signExtend(raw, sig.bit_length)) : static_cast<double>(raw);
      return signed_raw * sig.factor + sig.offset;
    }
  }
  return std::nullopt;
}

bool DbcProtocol::encodeSignal(CanFrame & frame, const std::string & signal_name, double physical_value, bool clamp)
{
  const auto msg_it = kMessageById.find(frame.can_id);
  if (msg_it == kMessageById.end()) { return false; }

  for (const auto & sig : msg_it->second.signals) {
    if (sig.name == signal_name) {
      double value = physical_value;
      if (clamp) {
        if (value < sig.min_value) value = sig.min_value;
        if (value > sig.max_value) value = sig.max_value;
      }
      const double raw_float = (value - sig.offset) / sig.factor;
      int64_t raw_signed = static_cast<int64_t>(std::llround(raw_float));
      const uint64_t max_unsigned = (sig.bit_length >= 64) ? std::numeric_limits<uint64_t>::max() :
        ((1ULL << sig.bit_length) - 1ULL);
      uint64_t raw = 0;
      if (sig.value_type == ValueType::Signed) {
        const int64_t min_signed = -(1LL << (sig.bit_length - 1));
        const int64_t max_signed = (1LL << (sig.bit_length - 1)) - 1LL;
        if (raw_signed < min_signed) raw_signed = min_signed;
        if (raw_signed > max_signed) raw_signed = max_signed;
        raw = static_cast<uint64_t>(raw_signed) & max_unsigned;
      } else {
        if (raw_signed < 0) raw_signed = 0;
        raw = static_cast<uint64_t>(raw_signed);
        if (raw > max_unsigned) raw = max_unsigned;
      }
      if (sig.byte_order == ByteOrder::Intel) {
        insertIntel(frame.data, sig.start_bit, sig.bit_length, raw);
      } else {
        insertMotorola(frame.data, sig.start_bit, sig.bit_length, raw);
      }
      return true;
    }
  }
  return false;
}

uint64_t DbcProtocol::extractIntel(const std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length)
{
  uint64_t value = 0;
  for (uint16_t i = 0; i < bit_length; ++i) {
    const uint16_t bit = start_bit + i;
    const uint8_t bit_val = static_cast<uint8_t>((data[bit / 8] >> (bit % 8)) & 0x1U);
    value |= (static_cast<uint64_t>(bit_val) << i);
  }
  return value;
}

uint64_t DbcProtocol::extractMotorola(const std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length)
{
  uint64_t value = 0;
  int bit = start_bit;
  for (uint16_t i = 0; i < bit_length; ++i) {
    const int byte_idx = bit / 8;
    const int bit_in_byte = 7 - (bit % 8);
    const uint8_t bit_val = static_cast<uint8_t>((data[byte_idx] >> bit_in_byte) & 0x1U);
    value = (value << 1) | bit_val;
    bit = (bit % 8 == 0) ? (bit + 15) : (bit - 1);
  }
  return value;
}

void DbcProtocol::insertIntel(std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length, uint64_t raw_value)
{
  for (uint16_t i = 0; i < bit_length; ++i) {
    const uint16_t bit = start_bit + i;
    const uint8_t bit_val = static_cast<uint8_t>((raw_value >> i) & 0x1U);
    data[bit / 8] = static_cast<uint8_t>((data[bit / 8] & ~(1U << (bit % 8))) | (bit_val << (bit % 8)));
  }
}

void DbcProtocol::insertMotorola(
  std::array<uint8_t, 8> & data, uint16_t start_bit, uint16_t bit_length, uint64_t raw_value)
{
  int bit = start_bit;
  for (uint16_t i = 0; i < bit_length; ++i) {
    const uint16_t src_idx = bit_length - 1 - i;
    const uint8_t bit_val = static_cast<uint8_t>((raw_value >> src_idx) & 0x1U);
    const int byte_idx = bit / 8;
    const int bit_in_byte = 7 - (bit % 8);
    data[byte_idx] = static_cast<uint8_t>((data[byte_idx] & ~(1U << bit_in_byte)) | (bit_val << bit_in_byte));
    bit = (bit % 8 == 0) ? (bit + 15) : (bit - 1);
  }
}

int64_t DbcProtocol::signExtend(uint64_t raw_value, uint16_t bit_length)
{
  if (bit_length == 0 || bit_length >= 64) {
    return static_cast<int64_t>(raw_value);
  }
  const uint64_t sign_bit = 1ULL << (bit_length - 1);
  if ((raw_value & sign_bit) != 0ULL) {
    raw_value |= (~0ULL << bit_length);
  }
  return static_cast<int64_t>(raw_value);
}

}  // namespace chassis_driver
