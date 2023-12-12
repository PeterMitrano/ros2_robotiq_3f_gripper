// Copyright (c) 2023 Peter Mitrano
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <serial/serial.h>

#include <algorithm>
#include <cmath>
#include <iostream>

#include <robotiq_3f_driver/default_driver.hpp>

#include <robotiq_3f_driver/crc_utils.hpp>
#include <robotiq_3f_driver/data_utils.hpp>
#include <robotiq_3f_driver/default_driver_utils.hpp>
#include <robotiq_3f_driver/driver_exception.hpp>

#include <rclcpp/logging.hpp>

// +-----+-------------------------------+-------------------------------+
// |     | Gripper Input Registers       | Gripper Output Registers      |
// +-----+-------------------------------+-------------------------------+
// | Idx | Address | Function            | Address | Function            |
// +-----+---------+---------------------+---------+---------------------+
// |   0 | 0x03E8  | Action Request      | 0x07D0  | Gripper Status      |
// |   1 | 0x03E9  | Gripper Options     | 0x07D1  | Object Detection    |
// |   2 | 0x03EA  | Empty               | 0x07D2  | Fault status        |
// |   3 | 0x03EB  | Finger A Position   | 0x07D3  | Finger A Req Echo   |
// |   4 | 0x03EC  | Finger A Speed      | 0x07D4  | Finger A Position   |
// |   5 | 0x03ED  | Finger A Force      | 0x07D5  | Finger A Current    |
// |   6 | 0x03EE  | Finger B Position   | 0x07D6  | Finger B Req Echo   |
// |   7 | 0x03EF  | Finger B Speed      | 0x07D7  | Finger B Position   |
// |   8 | 0x03F0  | Finger B Force      | 0x07D8  | Finger B Current    |
// |   9 | 0x03F1  | Finger C Position   | 0x07D9  | Finger C Req Echo   |
// |  10 | 0x03F2  | Finger C Speed      | 0x07DA  | Finger C Position   |
// |  11 | 0x03F3  | Finger C Force      | 0x07DB  | Finger C Current    |
// |  12 | 0x03F4  | Scissor Position    | 0x07DC  | Scissor Req Echo    |
// |  13 | 0x03F5  | Scissor Speed       | 0x07DD  | Scissor Position    |
// |  14 | 0x03F6  | Scissor Force       | 0x07DE  | Scissor Current     |
// +-----+---------+---------------------+---------+---------------------+

namespace robotiq_3f_driver
{
const auto kLogger = rclcpp::get_logger("DefaultDriver");

// If the gripper connection is not stable we may want to try sending the command again.
constexpr auto kMaxRetries = 5;

constexpr uint16_t kActionRequestRegisterAddress = 0x03E8;
constexpr size_t kActivateResponseSize = 8;
constexpr size_t kDectivateResponseSize = 8;


DefaultDriver::DefaultDriver(std::unique_ptr<Serial> serial) : serial_{ std::move(serial) }
{
}

std::vector<uint8_t> DefaultDriver::send(const std::vector<uint8_t>& request, size_t response_size) const
{
  std::vector<uint8_t> response;
  response.reserve(response_size);

  int retry_count = 0;
  while (retry_count < kMaxRetries)
  {
    try
    {
      serial_->write(request);
      response = serial_->read(response_size);
      break;
    }
    catch (const serial::IOException& e)
    {
      RCLCPP_DEBUG(kLogger, "Resending the command because the previous attempt (%d of %d) failed: %s", retry_count + 1,
                   kMaxRetries, e.what());
      retry_count++;
    }
  }

  if (retry_count == kMaxRetries)
  {
    RCLCPP_ERROR(kLogger, "Reached maximum retries. Operation failed.");
    return {};
  }

  return response;
}

bool DefaultDriver::connect()
{
  serial_->open();
  return serial_->is_open();
}

void DefaultDriver::disconnect()
{
  serial_->close();
}

void DefaultDriver::activate()
{
  RCLCPP_INFO(kLogger, "Activate...");

  // built up the Action request register (see 4.4)
  uint8_t action_request_register = 0b00000000;
  default_driver_utils::set_gripper_activation(action_request_register, GripperActivationAction::ACTIVE);

  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
    data_utils::get_msb(kActionRequestRegisterAddress),
    data_utils::get_lsb(kActionRequestRegisterAddress),
    0x00,                     // Number of registers to write MSB.
    0x03,                     // Number of registers to write LSB.
    0x06,                     // Number of bytes to write.
    action_request_register,  // Action register.
    0x00,                     // Reserved.
    0x00,                     // Reserved.
  };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  auto response = send(request, kActivateResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to activate the gripper." };
  }
}

void DefaultDriver::deactivate()
{
  RCLCPP_INFO(kLogger, "Deactivate...");

  // See 4.7.6 Modbus RTU Example, step 1: Activation request in the manual.
  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
    data_utils::get_msb(kActionRequestRegisterAddress),
    data_utils::get_lsb(kActionRequestRegisterAddress),
    0x00,  // Number of registers to write MSB.
    0x03,  // Number of registers to write LSB.
    0x06,  // Number of bytes to write.
    0x00,  // Action register.
    0x00,  // Reserved.
    0x00,  // Reserved.
    0x00,  // Max absolute pressure.
    0x00,  // Gripper Timeout.
    0x00,  // Min absolute pressure
  };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  auto response = send(request, kDectivateResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to deactivate the gripper." };
  }
}

//void DefaultDriver::grip()
//{
//  RCLCPP_INFO(kLogger, "Gripping...");
//
//  uint8_t action_request_register = 0b00000000;
//  default_driver_utils::set_gripper_activation(action_request_register, GripperActivationAction::Activate);
//  default_driver_utils::set_gripper_mode(action_request_register, grasping_mode_);
//  default_driver_utils::set_gripper_regulate_action(action_request_register,
//                                                    GripperRegulateAction::FollowRequestedVacuumParameters);
//
//  const uint8_t grip_max_absolute_pressure =
//      static_cast<uint8_t>(grasping_mode_ == GripperMode::AdvancedMode ?
//                               std::clamp(std::round(grip_max_vacuum_pressure_ + kAtmosphericPressure),
//                                          kMinAbsolutePressure, kMaxAbsolutePressure) :
//                               kMinAbsolutePressure);
//  const uint8_t grip_min_absolute_pressure = static_cast<uint8_t>(std::clamp(
//      std::round(grip_min_vacuum_pressure_ + kAtmosphericPressure), kMinAbsolutePressure, kMaxAbsolutePressure));
//  const auto timeout = static_cast<uint8_t>(
//      std::chrono::duration_cast<std::chrono::duration<int, std::ratio<1, 10>>>(
//          std::clamp(grip_timeout_, std::chrono::milliseconds(kMinTimeout), std::chrono::milliseconds(kMaxTimeout)))
//          .count());
//
//  std::vector<uint8_t> request = {
//    slave_address_,
//    static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
//    data_utils::get_msb(kActionRequestRegisterAddress),
//    data_utils::get_lsb(kActionRequestRegisterAddress),
//    0x00,                        // Number of registers to write MSB.
//    0x03,                        // Number of registers to write LSB.
//    0x06,                        // Number of bytes to write.
//    action_request_register,     // Action register.
//    0x00,                        // Reserved.
//    0x00,                        // Reserved.
//    grip_max_absolute_pressure,  // Grip max absolute pressure.
//    timeout,                     // Gripper timeout (hundredths of a second).
//    grip_min_absolute_pressure   // Min absolute pressure
//  };
//
//  auto crc = crc_utils::compute_crc(request);
//  request.push_back(data_utils::get_msb(crc));
//  request.push_back(data_utils::get_lsb(crc));
//
//  auto response = send(request, kGripResponseSize);
//  if (response.empty())
//  {
//    throw DriverException{ "Failed to grip." };
//  }
//}
//
//void DefaultDriver::release()
//{
//  RCLCPP_INFO(kLogger, "Releasing...");
//
//  uint8_t action_request_register = 0b00000000;
//  default_driver_utils::set_gripper_activation(action_request_register, GripperActivationAction::Activate);
//  default_driver_utils::set_gripper_mode(action_request_register, grasping_mode_);
//  default_driver_utils::set_gripper_regulate_action(action_request_register,
//                                                    GripperRegulateAction::FollowRequestedVacuumParameters);
//
//  const uint8_t release_absolute_pressure = static_cast<uint8_t>(kMaxAbsolutePressure);
//  const uint8_t grip_min_absolute_pressure = static_cast<uint8_t>(std::clamp(
//      std::round(grip_min_vacuum_pressure_ + kAtmosphericPressure), kMinAbsolutePressure, kMaxAbsolutePressure));
//  const auto timeout = static_cast<uint8_t>(
//      std::chrono::duration_cast<std::chrono::duration<int, std::ratio<1, 10>>>(
//          std::clamp(release_timeout_, std::chrono::milliseconds(kMinTimeout), std::chrono::milliseconds(kMaxTimeout)))
//          .count());
//
//  std::vector<uint8_t> request = {
//    slave_address_,
//    static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
//    data_utils::get_msb(kActionRequestRegisterAddress),
//    data_utils::get_lsb(kActionRequestRegisterAddress),
//    0x00,                       // Number of registers to write MSB.
//    0x03,                       // Number of registers to write LSB.
//    0x06,                       // Number of bytes to write.
//    action_request_register,    // Action register.
//    0x00,                       // Reserved.
//    0x00,                       // Reserved.
//    release_absolute_pressure,  // Grip max absolute pressure.
//    timeout,                    // Gripper timeout (hundredths of a second).
//    grip_min_absolute_pressure  // Min absolute pressure
//  };
//
//  auto crc = crc_utils::compute_crc(request);
//  request.push_back(data_utils::get_msb(crc));
//  request.push_back(data_utils::get_lsb(crc));
//
//  auto response = send(request, kReleaseResponseSize);
//  if (response.empty())
//  {
//    throw DriverException{ "Failed to release." };
//  }
//}
//
//void DefaultDriver::set_slave_address(uint8_t slave_address)
//{
//  slave_address_ = slave_address;
//}
//
//void DefaultDriver::set_mode(GripperMode gripper_mode)
//{
//  if (gripper_mode == GripperMode::Unknown)
//  {
//    RCLCPP_ERROR(kLogger, "Invalid gripper mode: %s",
//                 default_driver_utils::gripper_mode_to_string(gripper_mode).c_str());
//    return;
//  }
//  grasping_mode_ = gripper_mode;
//}
//
//GripperStatus DefaultDriver::get_status()
//{
//  std::vector<uint8_t> request = {
//    slave_address_,
//    static_cast<uint8_t>(default_driver_utils::FunctionCode::ReadInputRegisters),
//    data_utils::get_msb(kGripperStatusRegister),
//    data_utils::get_lsb(kGripperStatusRegister),
//    0x00,  // Number of registers to read MSB
//    0x03   // Number of registers to read LSB
//  };
//  auto crc = crc_utils::compute_crc(request);
//  request.push_back(data_utils::get_msb(crc));
//  request.push_back(data_utils::get_lsb(crc));
//
//  auto response = send(request, kGetStatusResponseSize);
//  if (response.empty())
//  {
//    throw DriverException{ "Failed to read the status." };
//  }
//
//  // The content of the requested registers starts from byte 3.
//
//  GripperStatus status;
//  status.gripper_activation_action = default_driver_utils::get_gripper_activation_action(response[3]);
//  status.gripper_mode = default_driver_utils::get_gripper_mode(response[3]);
//  status.gripper_regulate_action = default_driver_utils::get_gripper_regulate_action(response[3]);
//  status.gripper_activation_status = default_driver_utils::get_gripper_activation_status(response[3]);
//  status.object_detection_status = default_driver_utils::get_object_detection_status(response[3]);
//  status.actuator_status = default_driver_utils::get_actuator_status(response[4]);
//  status.gripper_fault_status = default_driver_utils::get_gripper_fault_status(response[5]);
//
//  // The requested pressure level in kPa:
//  status.max_vacuum_pressure = static_cast<float>(response[6]) - kAtmosphericPressure;
//
//  // The actual pressure measured kPa:
//  status.actual_vacuum_pressure = static_cast<float>(response[7]) - kAtmosphericPressure;
//
//  return status;
//}
}  // namespace robotiq_3f_driver
