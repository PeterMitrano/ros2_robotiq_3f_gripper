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

#include <gtest/gtest.h>

#include <robotiq_3f_driver/data_utils.hpp>
#include <robotiq_3f_driver/default_driver.hpp>
#include <robotiq_3f_driver/default_driver_utils.hpp>

#include <mock/mock_serial.hpp>

namespace robotiq_3f_driver::test
{
using ::testing::_;
using ::testing::Return;
using ::testing::SaveArg;

/**
 * Here we test the driver activation command.
 * We mock the serial interface, send an activate command to the driver and
 * check that the sequence of bytes received by the serial interface is
 * correctly generated.
 */
TEST(TestDefaultDriver, activate)
{
  uint8_t slave_address = 0x09;

  // clang-format off

  const std::vector<uint8_t> expected_request{
    slave_address,
    static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
    0x03, 0xE8,  // Address of the first requested register - MSB, LSB.
    0x00, 0x01,  // Number of registers requested - MSB, LSB.
    0x02,        // Number of data bytes to follow.
    0x01, 0x00,   // Action Register - MSB, LSB.
    0xE4, 0x28   // CRC-16 - MSB, LSB.
  };

  const std::vector<uint8_t> expected_response {
    slave_address,
    static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
    0x03, 0xE8,  // Address of the first requested register - MSB, LSB.
    0x00, 0x03,  // Number of registers requested - MSB, LSB.
    0x01, 0x30   // CRC-16 - MSB, LSB.
  };

  std::vector<uint8_t> actual_request;
  std::vector<uint8_t> actual_response;
  auto serial = std::make_unique<MockSerial>();
  EXPECT_CALL(*serial, write(_)).WillOnce(SaveArg<0>(&actual_request));
  EXPECT_CALL(*serial, read(_)).WillOnce(Return(expected_response));

  auto driver = std::make_unique<robotiq_3f_driver::DefaultDriver>(std::move(serial));
  driver->set_slave_address(slave_address);

  driver->activate();

  ASSERT_EQ(data_utils::to_hex(actual_request), data_utils::to_hex(expected_request));
}

// TODO: add more tests for the other functions of DefaultDriver


}  // namespace robotiq_3f_driver::test
