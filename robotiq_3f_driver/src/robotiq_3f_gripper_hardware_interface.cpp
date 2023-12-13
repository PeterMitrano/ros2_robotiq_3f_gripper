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

#include <chrono>
#include <cmath>
#include <optional>
#include <thread>

#include <robotiq_3f_driver/robotiq_3f_gripper_hardware_interface.hpp>

#include <robotiq_3f_driver/default_driver_factory.hpp>
#include <robotiq_3f_driver/hardware_interface_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace robotiq_3f_driver
{
const auto kLogger = rclcpp::get_logger("Robotiq3fGripperHardwareInterface");

constexpr auto kGripCommandInterfaceName = "cmd";
constexpr auto kGripStateInterfaceName = "state";

constexpr auto kObjectDetectionStateInterfaceName = "object_detection_status";

constexpr auto kGripperCommsLoopPeriod = std::chrono::milliseconds{ 10 };

using robotiq_3f_driver::hardware_interface_utils::get_gpios_command_interface;
using robotiq_3f_driver::hardware_interface_utils::get_gpios_state_interface;
using robotiq_3f_driver::hardware_interface_utils::get_joints_state_interface;
using robotiq_3f_driver::hardware_interface_utils::is_false;
using robotiq_3f_driver::hardware_interface_utils::is_true;

Robotiq3fGripperHardwareInterface::Robotiq3fGripperHardwareInterface()
{
  driver_factory_ = std::make_unique<DefaultDriverFactory>();
}

// This constructor is use for testing only.
Robotiq3fGripperHardwareInterface::Robotiq3fGripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory)
  : driver_factory_{ std::move(driver_factory) }
{
}

Robotiq3fGripperHardwareInterface::~Robotiq3fGripperHardwareInterface()
{
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Robotiq3fGripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_DEBUG(kLogger, "on_init");
  try
  {
    // Store hardware info for later use.

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    for (uint i = 0; i < info_.gpios.size(); i++)
    {
      auto gpio = info_.gpios[i];
    }

    driver_ = driver_factory_->create(info);

    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Cannot initialize the Robotiq 3f gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Robotiq3fGripperHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_configure");
  try
  {
    if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Open the serial port and handshake.
    bool connected = driver_->connect();
    if (!connected)
    {
      RCLCPP_ERROR(kLogger, "Cannot connect to the Robotiq 3f gripper");
      return CallbackReturn::ERROR;
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Cannot configure the Robotiq 3f gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Robotiq3fGripperHardwareInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  try
  {
    state_interfaces.emplace_back("finger_a_cmd_echo", hardware_interface::HW_IF_POSITION,
                                  &status_.finger_a_position_cmd_echo);
    state_interfaces.emplace_back("finger_b_cmd_echo", hardware_interface::HW_IF_POSITION,
                                  &status_.finger_b_position_cmd_echo);
    state_interfaces.emplace_back("finger_c_cmd_echo", hardware_interface::HW_IF_POSITION,
                                  &status_.finger_c_position_cmd_echo);
    state_interfaces.emplace_back("scissor_cmd_echo", hardware_interface::HW_IF_POSITION,
                                  &status_.scissor_position_cmd_echo);
    state_interfaces.emplace_back("finger_a", hardware_interface::HW_IF_POSITION, &status_.finger_a_position);
    state_interfaces.emplace_back("finger_b", hardware_interface::HW_IF_POSITION, &status_.finger_b_position);
    state_interfaces.emplace_back("finger_c", hardware_interface::HW_IF_POSITION, &status_.finger_c_position);
    state_interfaces.emplace_back("scissor", hardware_interface::HW_IF_POSITION, &status_.scissor_position);
    state_interfaces.emplace_back("finger_a_current", hardware_interface::HW_IF_EFFORT, &status_.finger_a_current);
    state_interfaces.emplace_back("finger_b_current", hardware_interface::HW_IF_EFFORT, &status_.finger_b_current);
    state_interfaces.emplace_back("finger_c_current", hardware_interface::HW_IF_EFFORT, &status_.finger_c_current);
    state_interfaces.emplace_back("scissor_current", hardware_interface::HW_IF_EFFORT, &status_.scissor_current);

    // TODO: how do we publish the object detection status? It's not a double. So maybe create a message type and publish that?
  }
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return {};
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Robotiq3fGripperHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  try
  {
    command_interfaces.emplace_back("finger_a_position", hardware_interface::HW_IF_POSITION, &cmd_.finger_a_position);
    command_interfaces.emplace_back("finger_b_position", hardware_interface::HW_IF_POSITION, &cmd_.finger_b_position);
    command_interfaces.emplace_back("finger_c_position", hardware_interface::HW_IF_POSITION, &cmd_.finger_c_position);
    command_interfaces.emplace_back("scissor_position", hardware_interface::HW_IF_POSITION, &cmd_.scissor_position);
    command_interfaces.emplace_back("finger_a_velocity", hardware_interface::HW_IF_VELOCITY, &cmd_.finger_a_velocity);
    command_interfaces.emplace_back("finger_b_velocity", hardware_interface::HW_IF_VELOCITY, &cmd_.finger_b_velocity);
    command_interfaces.emplace_back("finger_c_velocity", hardware_interface::HW_IF_VELOCITY, &cmd_.finger_c_velocity);
    command_interfaces.emplace_back("scissor_velocity", hardware_interface::HW_IF_VELOCITY, &cmd_.scissor_velocity);
    command_interfaces.emplace_back("finger_a_force", hardware_interface::HW_IF_EFFORT, &cmd_.finger_a_force);
    command_interfaces.emplace_back("finger_b_force", hardware_interface::HW_IF_EFFORT, &cmd_.finger_b_force);
    command_interfaces.emplace_back("finger_c_force", hardware_interface::HW_IF_EFFORT, &cmd_.finger_c_force);
    command_interfaces.emplace_back("scissor_force", hardware_interface::HW_IF_EFFORT, &cmd_.scissor_force);
  }
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return {};
  }
  return command_interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Robotiq3fGripperHardwareInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_activate");
  try
  {
    driver_->activate();

    // The following thread will be responsible for communicating directly with the driver.
    communication_thread_is_running_.store(true);
    communication_thread_ = std::thread([this] { this->background_task(); });
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(kLogger, "Failed to activate the Robotiq 3f gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "Robotiq 3f Gripper successfully activated!");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Robotiq3fGripperHardwareInterface::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_deactivate");
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }
  try
  {
    driver_->deactivate();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to deactivate the Robotiq 3f gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(kLogger, "Robotiq 3f Gripper successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Robotiq3fGripperHardwareInterface::read([[maybe_unused]] const rclcpp::Time& time,
                                                                        [[maybe_unused]] const rclcpp::Duration& period)
{
  // Read doesn't need to do anything, since the backgorund thread is already copying data into the status_ struct,
  // which is bound to the state interface.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Robotiq3fGripperHardwareInterface::write([[maybe_unused]] const rclcpp::Time& time,
                                                                         [[maybe_unused]] const rclcpp::Duration& period)
{
  // Write doesn't need to do anything, since the command interface is bound to the cmd_ struct, which is copied into
  // the driver in the background thread.

  return hardware_interface::return_type::OK;
}

void Robotiq3fGripperHardwareInterface::background_task()
{
  // Read from and write to the gripper at a fixed rate set by kGripperCommsLoopPeriod.
  while (communication_thread_is_running_.load())
  {
    try
    {
      // Retrieve current status and update state interfaces, and copy the status into the status_ struct.
      // make sure to lock the mutex while we're reading and writing to the status_ struct.
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        status_ = driver_->get_full_status();
      }

      // Send the commands to the driver
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        driver_->write(cmd_);
      }
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(kLogger, "Error: %s", e.what());
    }
  }

  std::this_thread::sleep_for(kGripperCommsLoopPeriod);
}
}  // namespace robotiq_3f_driver

PLUGINLIB_EXPORT_CLASS(robotiq_3f_driver::Robotiq3fGripperHardwareInterface, hardware_interface::SystemInterface)
