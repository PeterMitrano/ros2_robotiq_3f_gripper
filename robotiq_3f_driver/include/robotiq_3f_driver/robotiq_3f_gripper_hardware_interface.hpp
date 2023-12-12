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

#pragma once

#include <atomic>
#include <thread>
#include <memory>
#include <vector>

#include <robotiq_3f_driver/default_driver_utils.hpp>
#include <robotiq_3f_driver/driver.hpp>
#include <robotiq_3f_driver/driver_factory.hpp>
#include <robotiq_3f_driver/visibility_control.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>

#include "visibility_control.hpp"

namespace robotiq_3f_driver
{

// We use this structure to hold our command interface values.
struct Command
{
  double finger_a_position;
  double finger_b_position;
  double finger_c_position;
  double scissor_position;
};

// We use this structure to hold a thread-safe copy of our command interface values.
struct SafeCommand
{
  std::atomic<Command> cmd;
};

// We use this structure to hold our state interface values.
struct State
{
  FullGripperStatus status;

  double finger_a_position_cmd_echo;
  double finger_a_position;
  double finger_a_current;
  ObjectDetectionStatus finger_a_object_detection_status;

  double finger_b_position_cmd_echo;
  double finger_b_position;
  double finger_b_current;
  ObjectDetectionStatus finger_b_object_detection_status;

  double finger_c_position_cmd_echo;
  double finger_c_position;
  double finger_c_current;
  ObjectDetectionStatus finger_c_object_detection_status;

  double scissor_position_cmd_echo;
  double scissor_position;
  double scissor_current;
  ObjectDetectionStatus scissor_object_detection_status;
};

// We use this structure to hold a thread-safe copy of our state interface values.
struct SafeState
{
  std::atomic<State> state;
};

class Robotiq3fGripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ROBOTIC_3FDRIVER_PUBLIC
  Robotiq3fGripperHardwareInterface();

  ROBOTIC_3FDRIVER_PUBLIC
  ~Robotiq3fGripperHardwareInterface();

  /**
   * Constructor with given command interface. This method is used for testing.
   * @param driver_factory The interface to send commands and queries to
   * the hardware.
   */
  explicit Robotiq3fGripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory);

  /**
   * Defines aliases and static functions for using the Class with shared_ptrs.
   * With such definitions, the control manager can instantiate the class with
   * auto hardware_interface = StepitHardware::make_shared();
   */
  RCLCPP_SHARED_PTR_DEFINITIONS(Robotiq3fGripperHardwareInterface)

  /**
   * Initialization of the hardware interface from data parsed from the
   * robot's URDF.
   * @param hardware_info Structure with data from URDF.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  ROBOTIC_3FDRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * Connect to the hardware.
   * @param previous_state The previous state.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  ROBOTIC_3FDRIVER_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * This method exposes position and velocity of joints for reading.
   */
  ROBOTIC_3FDRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * This method exposes the joints targets for writing.
   */
  ROBOTIC_3FDRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * This method is invoked when the hardware is connected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  ROBOTIC_3FDRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * This method is invoked when the hardware is disconnected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  ROBOTIC_3FDRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Read data from the hardware.
   */
  ROBOTIC_3FDRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * Write data to hardware.
   */
  ROBOTIC_3FDRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Interface to interact with the hardware using the serial port.
  std::unique_ptr<Driver> driver_;

  // Factory to create the interface to interact with the hardware using the serial port.
  std::unique_ptr<DriverFactory> driver_factory_;

  // We use a thread to read/write to the driver so that we don't block the hardware_interface read/write.
  std::thread communication_thread_;
  std::atomic<bool> communication_thread_is_running_;
  void background_task();

  // This stores thread-safe copies of command and state, since we use a background thread to constantly read/write to
  // the hardware
  SafeCommand safe_cmd_;
  SafeState safe_state_;

};
}  // namespace robotiq_3f_driver
