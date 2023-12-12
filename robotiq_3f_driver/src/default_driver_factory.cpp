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

#include <cmath>

#include <robotiq_3f_driver/default_driver_factory.hpp>

#include <robotiq_3f_driver/data_utils.hpp>
#include <robotiq_3f_driver/default_driver.hpp>
#include <robotiq_3f_driver/default_driver_utils.hpp>
#include <robotiq_3f_driver/fake/fake_driver.hpp>
#include <robotiq_3f_driver/default_serial.hpp>
#include <robotiq_3f_driver/default_serial_factory.hpp>

#include <rclcpp/logging.hpp>

namespace robotiq_3f_driver
{

const auto kLogger = rclcpp::get_logger("DefaultDriverFactory");

constexpr auto kSlaveAddressParamName = "slave_address";
constexpr uint8_t kSlaveAddressParamDefault = 0x9;

constexpr auto kUseDummyParamName = "use_dummy";
constexpr auto kUseDummyParamDefault = "False";  // TODO(kineticsystem): make this so it's not case sensitive

using data_utils::to_lower;

std::unique_ptr<robotiq_3f_driver::Driver>
robotiq_3f_driver::DefaultDriverFactory::create(const hardware_interface::HardwareInfo& info) const
{
  RCLCPP_INFO(kLogger, "Reading slave_address...");
  // Convert base-16 address stored as a string (for example, "0x9") into an integer
  const uint8_t slave_address =
      info.hardware_parameters.count(kSlaveAddressParamName) ?
          static_cast<uint8_t>(std::stoul(info.hardware_parameters.at(kSlaveAddressParamName), nullptr, 16)) :
          kSlaveAddressParamDefault;
  RCLCPP_INFO(kLogger, "slave_address: %d", slave_address);

  auto driver = create_driver(info);
  driver->set_slave_address(slave_address);
  return driver;
}

std::unique_ptr<Driver> DefaultDriverFactory::create_driver(const hardware_interface::HardwareInfo& info) const
{
  // We give the user an option to start up a dummy gripper for testing purposes.
  if (info.hardware_parameters.count(kUseDummyParamName) &&
      to_lower(info.hardware_parameters.at(kUseDummyParamName)) != to_lower(kUseDummyParamDefault))
  {
    RCLCPP_INFO(kLogger, "You are connected to a dummy driver, not a real hardware.");
    return std::make_unique<FakeDriver>();
  }
  else
  {
    auto serial = DefaultSerialFactory().create(info);
    return std::make_unique<DefaultDriver>(std::move(serial));
  }
}
}  // namespace robotiq_3f_driver
