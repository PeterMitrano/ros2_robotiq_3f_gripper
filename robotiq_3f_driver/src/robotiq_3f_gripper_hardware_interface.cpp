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
#include <robotiq_3f_transmission_plugins/individual_control_transmission_loader.hpp>

#include <robotiq_3f_driver/default_driver_factory.hpp>
#include <robotiq_3f_driver/hardware_interface_utils.hpp>

#include <transmission_interface/transmission_interface_exception.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace robotiq_3f_driver
{
const auto kLogger = rclcpp::get_logger("Robotiq3fGripperHardwareInterface");

constexpr auto kGripperCommsLoopPeriod = std::chrono::milliseconds{ 10 };
constexpr auto kModeInterface{ "mode" };

using robotiq_3f_driver::hardware_interface_utils::get_gpios_command_interface;
using robotiq_3f_driver::hardware_interface_utils::get_gpios_state_interface;
using robotiq_3f_driver::hardware_interface_utils::get_joints_state_interface;
using robotiq_3f_driver::hardware_interface_utils::is_false;
using robotiq_3f_driver::hardware_interface_utils::is_true;

Robotiq3fGripperHardwareInterface::Robotiq3fGripperHardwareInterface()
{
  driver_factory_ = std::make_unique<DefaultDriverFactory>();

  rclcpp::on_shutdown([this] { stop(); });
}

// This constructor is use for testing only.
Robotiq3fGripperHardwareInterface::Robotiq3fGripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory)
  : driver_factory_{ std::move(driver_factory) }
{
  rclcpp::on_shutdown([this] { stop(); });
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

    driver_ = driver_factory_->create(info);

    if (info_.transmissions.empty())
    {
      RCLCPP_ERROR_STREAM(kLogger, "No transmissions found in the robotiq_3f_gripper_hardware_interface."
                                       << " You need to list the IndividualControlTransmission ");
      return CallbackReturn::ERROR;
    }

    robotiq_3f_transmission_plugins::IndividualControlTransmissionLoader transmission_loader;
    transmission_ = transmission_loader.load(info_.transmissions[0]);

    std::vector<transmission_interface::ActuatorHandle> actuator_handles{
      { "finger_a", hardware_interface::HW_IF_POSITION, &act_state_.finger_a },
      { "finger_b", hardware_interface::HW_IF_POSITION, &act_state_.finger_b },
      { "finger_c", hardware_interface::HW_IF_POSITION, &act_state_.finger_c },
      { "scissor", hardware_interface::HW_IF_POSITION, &act_state_.scissor },
    };

    std::vector<transmission_interface::JointHandle> joint_handles{
      { "finger_a_joint_1", hardware_interface::HW_IF_POSITION, &trn_output_.finger_a_joint_1 },
      { "finger_a_joint_2", hardware_interface::HW_IF_POSITION, &trn_output_.finger_a_joint_2 },
      { "finger_a_joint_3", hardware_interface::HW_IF_POSITION, &trn_output_.finger_a_joint_3 },
      { "finger_b_joint_1", hardware_interface::HW_IF_POSITION, &trn_output_.finger_b_joint_1 },
      { "finger_b_joint_2", hardware_interface::HW_IF_POSITION, &trn_output_.finger_b_joint_2 },
      { "finger_b_joint_3", hardware_interface::HW_IF_POSITION, &trn_output_.finger_b_joint_3 },
      { "finger_c_joint_1", hardware_interface::HW_IF_POSITION, &trn_output_.finger_c_joint_1 },
      { "finger_c_joint_2", hardware_interface::HW_IF_POSITION, &trn_output_.finger_c_joint_2 },
      { "finger_c_joint_3", hardware_interface::HW_IF_POSITION, &trn_output_.finger_c_joint_3 },
      { "palm_finger_c_joint", hardware_interface::HW_IF_POSITION, &trn_output_.palm_finger_c_joint },
      { "palm_finger_b_joint", hardware_interface::HW_IF_POSITION, &trn_output_.palm_finger_b_joint },
    };

    // Set up the transmission
    try
    {
      transmission_->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException& exc)
    {
      RCLCPP_FATAL_STREAM(kLogger, "Error while configuring transmission: " << exc.what());
      return CallbackReturn::ERROR;
    }

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
    // Here we bind the state interface to the trn_output_ struct, which is updated in the background thread.
    state_interfaces.emplace_back("finger_a_joint_1", hardware_interface::HW_IF_POSITION, &trn_output_.finger_a_joint_1);
    state_interfaces.emplace_back("finger_a_joint_2", hardware_interface::HW_IF_POSITION, &trn_output_.finger_a_joint_2);
    state_interfaces.emplace_back("finger_a_joint_3", hardware_interface::HW_IF_POSITION, &trn_output_.finger_a_joint_3);
    state_interfaces.emplace_back("finger_b_joint_1", hardware_interface::HW_IF_POSITION, &trn_output_.finger_b_joint_1);
    state_interfaces.emplace_back("finger_b_joint_2", hardware_interface::HW_IF_POSITION, &trn_output_.finger_b_joint_2);
    state_interfaces.emplace_back("finger_b_joint_3", hardware_interface::HW_IF_POSITION, &trn_output_.finger_b_joint_3);
    state_interfaces.emplace_back("finger_c_joint_1", hardware_interface::HW_IF_POSITION, &trn_output_.finger_c_joint_1);
    state_interfaces.emplace_back("finger_c_joint_2", hardware_interface::HW_IF_POSITION, &trn_output_.finger_c_joint_2);
    state_interfaces.emplace_back("finger_c_joint_3", hardware_interface::HW_IF_POSITION, &trn_output_.finger_c_joint_3);
    state_interfaces.emplace_back("palm_finger_c_joint", hardware_interface::HW_IF_POSITION,
                                  &trn_output_.palm_finger_c_joint);
    state_interfaces.emplace_back("palm_finger_b_joint", hardware_interface::HW_IF_POSITION,
                                  &trn_output_.palm_finger_b_joint);

    // Also bind to the position of the "actuators"
    state_interfaces.emplace_back("finger_a", hardware_interface::HW_IF_POSITION, &act_state_.finger_a);
    state_interfaces.emplace_back("finger_b", hardware_interface::HW_IF_POSITION, &act_state_.finger_b);
    state_interfaces.emplace_back("finger_c", hardware_interface::HW_IF_POSITION, &act_state_.finger_c);
    state_interfaces.emplace_back("scissor", hardware_interface::HW_IF_POSITION, &act_state_.scissor);
    // There is no velocity, but there is effort information
    state_interfaces.emplace_back("finger_a", hardware_interface::HW_IF_EFFORT, &joint_current_state_.finger_a);
    state_interfaces.emplace_back("finger_b", hardware_interface::HW_IF_EFFORT, &joint_current_state_.finger_b);
    state_interfaces.emplace_back("finger_c", hardware_interface::HW_IF_EFFORT, &joint_current_state_.finger_c);
    state_interfaces.emplace_back("scissor", hardware_interface::HW_IF_EFFORT, &joint_current_state_.scissor);

    // TODO: how do we publish the object detection status? It's not a double. So maybe create a message type and publish that?
    state_interfaces.emplace_back("finger_a", "object_detection_status", &object_detection_state_.finger_a);
    state_interfaces.emplace_back("finger_b", "object_detection_status", &object_detection_state_.finger_b);
    state_interfaces.emplace_back("finger_c", "object_detection_status", &object_detection_state_.finger_c);
    state_interfaces.emplace_back("scissor", "object_detection_status", &object_detection_state_.scissor);

    state_interfaces.emplace_back("simple_control_grasp_mode", kModeInterface, &grasp_mode_);
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
    command_interfaces.emplace_back("finger_a", hardware_interface::HW_IF_POSITION, &independent_cmd_.finger_a_position);
    command_interfaces.emplace_back("finger_b", hardware_interface::HW_IF_POSITION, &independent_cmd_.finger_b_position);
    command_interfaces.emplace_back("finger_c", hardware_interface::HW_IF_POSITION, &independent_cmd_.finger_c_position);
    command_interfaces.emplace_back("scissor", hardware_interface::HW_IF_POSITION, &independent_cmd_.scissor_position);

    command_interfaces.emplace_back("simple", hardware_interface::HW_IF_POSITION, &simple_cmd_.position);
    command_interfaces.emplace_back("simple", hardware_interface::HW_IF_VELOCITY, &simple_cmd_.speed);
    command_interfaces.emplace_back("simple", hardware_interface::HW_IF_EFFORT, &simple_cmd_.force);

    // using POSITION here is such a hack
    command_interfaces.emplace_back("grasping_mode", hardware_interface::HW_IF_POSITION, &simple_cmd_.grasping_mode);
    command_interfaces.emplace_back("use_independent_control", hardware_interface::HW_IF_POSITION, &use_independent_control_);
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
  try
  {
    stop();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to deactivate the Robotiq 3f gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(kLogger, "Robotiq 3f Gripper successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void Robotiq3fGripperHardwareInterface::stop()
{
  RCLCPP_DEBUG(kLogger, "stop");
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }
  driver_->deactivate();
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
  // Write doesn't need to do anything, since the command interface is bound to the independent_cmd_ struct, which is
  // copied into the driver in the background thread.

  return hardware_interface::return_type::OK;
}

void Robotiq3fGripperHardwareInterface::background_task()
{
  // Read from and send_independent_control_command to the gripper at a fixed rate set by kGripperCommsLoopPeriod.
  while (communication_thread_is_running_.load())
  {
    try
    {
      // Retrieve current status and copy the status into the status_ struct, which is bound to the state interface.
      // make sure to lock the mutex while we're writing to the status_ struct.
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto const status = driver_->get_full_status();

        act_state_.finger_a = default_driver_utils::uint8_to_double(status.finger_a_position);
        act_state_.finger_b = default_driver_utils::uint8_to_double(status.finger_b_position);
        act_state_.finger_c = default_driver_utils::uint8_to_double(status.finger_c_position);
        act_state_.scissor = default_driver_utils::uint8_to_double(status.scissor_position);

        joint_current_state_.finger_a = default_driver_utils::uint8_to_double(status.finger_a_current);
        joint_current_state_.finger_b = default_driver_utils::uint8_to_double(status.finger_b_current);
        joint_current_state_.finger_c = default_driver_utils::uint8_to_double(status.finger_c_current);
        joint_current_state_.scissor = default_driver_utils::uint8_to_double(status.scissor_current);

        object_detection_state_.finger_a =
            default_driver_utils::object_detection_status_to_double(status.finger_a_object_detection_status);
        object_detection_state_.finger_b =
            default_driver_utils::object_detection_status_to_double(status.finger_b_object_detection_status);
        object_detection_state_.finger_c =
            default_driver_utils::object_detection_status_to_double(status.finger_c_object_detection_status);
        object_detection_state_.scissor =
            default_driver_utils::object_detection_status_to_double(status.scissor_object_detection_status);

        grasp_mode_ = default_driver_utils::grasping_mode_to_double(status.mode_status);

        // use the transmissions to convert the status to the state
        transmission_->actuator_to_joint();
      }

      // Send the commands to the driver
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if (static_cast<bool>(use_independent_control_))
        {
          driver_->send_independent_control_command(independent_cmd_);
        }
        else
        {
          auto const grasping_mode = default_driver_utils::double_to_grasping_mode(simple_cmd_.grasping_mode);
          driver_->send_simple_control_command(grasping_mode, simple_cmd_.position, simple_cmd_.speed,
                                               simple_cmd_.force);
        }
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
