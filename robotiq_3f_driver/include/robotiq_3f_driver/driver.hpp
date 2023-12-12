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

#include <chrono>
#include <string>

namespace robotiq_3f_driver
{

enum class GripperActivationAction
{
  RESET,
  ACTIVE
};
// They are the same, but we want to be explicit about what we are doing
using GripperActivationStatus = GripperActivationAction;

enum class GraspingMode
{
  BASIC,
  PINCH,
  WIDE,
  SCISSOR
};

enum class GoTo
{
  STOP,
  GO_TO
};

enum class ActionStatus
{
  STOPPED,
  MOVING
};

enum class MotionStatus // gSTA
{
  IN_MOTION,
  STOPPED_ONE_OR_TWO_URNEACHED,
  STOPPED_THREE_UNREACHED,
  STOPPED_REACHED
};

enum class ObjectDetectionStatus
{
  MOVING,
  OBJECT_DETECTED_OPENING,
  OBJECT_DETECTED_CLOSING,
  AT_REQUESTED_POSITION
};

enum class GripperStatus
{
  RESET,
  ACTIVATION_IN_PROGRESS,
  MODE_CHANGE_IN_PROGRESS,
  ACTIVATED
};

enum class GripperFaultStatus
{
  // priority faults
  ACTION_DELAYED_REACTIVATION_NEEDED,
  ACTION_DELAYED_MODE_CHANGE_NEEDED,
  ACTIVATION_BIT_NOT_SET,
  // minor faults
  NOT_READY,
  MODE_CHANGE_SCISSOR_MOTION_ERROR,
  AUTO_RELEASE_IN_PROGRESS,
  // major faults
  ACTIVATION_FAULT,
  MODE_CHANGE_FAULT,
  AUTOMATIC_RELEASE_FAULT,
};

struct FullGripperStatus
{
  GripperActivationStatus activation_status;  // an echo of the activation command
  GraspingMode mode_status;
  GoTo go_to_status;
  GripperStatus gripper_status;
  MotionStatus motion_status;
};

/**
 * This is the interface of the driver to control the 3f Gripper.
 * The Driver interface can be easily mocked for testing or implemented to
 * fake the behavior of the real hardware.
 */
class Driver
{
public:
  virtual void set_slave_address(uint8_t slave_address) = 0;

  /** Connect to the gripper serial connection. */
  virtual bool connect() = 0;

  /** Disconnect from the gripper serial connection. */
  virtual void disconnect() = 0;

  /**
   * @brief Activates the gripper.
   * @throw serial::IOException on failure to successfully communicate with gripper port
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivates the gripper.
   * @throw serial::IOException on failure to successfully communicate with gripper port
   */
  virtual void deactivate() = 0;

  virtual FullGripperStatus get_full_status() = 0;
  /**
   * gACT: initialization status
   */
  virtual uint8_t get_activation_status() = 0;
  // TODO: add register names
  virtual uint8_t get_operation_mode_status() = 0;
  virtual uint8_t get_action_status() = 0;
  virtual uint8_t get_gripper_status() = 0;
  virtual uint8_t get_motion_status() = 0;
  virtual uint8_t get_fault_status() = 0;
  virtual uint8_t get_finger_a_commanded_position() = 0;
  virtual uint8_t get_finger_a_position() = 0;
  virtual uint8_t get_finger_a_current() = 0;
  virtual uint8_t get_finger_a_object_status() = 0;
  virtual uint8_t get_finger_b_commanded_position() = 0;
  virtual uint8_t get_finger_b_position() = 0;
  virtual uint8_t get_finger_b_current() = 0;
  virtual uint8_t get_finger_b_object_status() = 0;
  virtual uint8_t get_finger_c_commanded_position() = 0;
  virtual uint8_t get_finger_c_position() = 0;
  virtual uint8_t get_finger_c_current() = 0;
  virtual uint8_t get_finger_c_object_status() = 0;
  virtual uint8_t get_scissor_commanded_position() = 0;
  virtual uint8_t get_scissor_position() = 0;
  virtual uint8_t get_scissor_current() = 0;
  virtual uint8_t get_scissor_object_status() = 0;

  virtual void set_activate(uint8_t activate) = 0;  // akin to "initialization"
  virtual void set_grasping_mode(uint8_t grasping_mode) = 0;
  virtual void set_go_to(uint8_t go_to) = 0;
  virtual void set_auto_release(uint8_t auto_release) = 0;
  virtual void set_individual_finger_control(uint8_t individual_finger_control) = 0;
  virtual void set_individual_scissor_control(uint8_t individual_scissor_control) = 0;
  virtual void set_finger_a_position(uint8_t pos) = 0;
  virtual void set_finger_a_speed(uint8_t speed) = 0;
  virtual void set_finger_a_force(uint8_t force) = 0;
  virtual void set_finger_b_position(uint8_t pos) = 0;
  virtual void set_finger_b_speed(uint8_t speed) = 0;
  virtual void set_finger_b_force(uint8_t force) = 0;
  virtual void set_finger_c_position(uint8_t pos) = 0;
  virtual void set_finger_c_speed(uint8_t speed) = 0;
  virtual void set_finger_c_force(uint8_t force) = 0;
  virtual void set_scissor_position(uint8_t pos) = 0;
  virtual void set_scissor_speed(uint8_t speed) = 0;
  virtual void set_scissor_force(uint8_t force) = 0;
};

}  // namespace robotiq_3f_driver
