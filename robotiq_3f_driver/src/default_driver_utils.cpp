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

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <unordered_map>

#include <robotiq_3f_driver/default_driver_utils.hpp>

namespace robotiq_3f_driver::default_driver_utils
{

//////////// Setter utils ////////////////////////////////////////////////////

/**
 * Sets bits in a register based on a bitmask and a set of bits.
 * @param reg Initial register value.
 * @param bitmask Mask that indicates which bits in the register should be modified.
 *        A '1' in a bit position indicates that the corresponding bit in the register
 *        will be modified, and a '0' means it will remain unchanged.
 * @param bits Bits to be set in the register. Only the bits that are '1' in the bitmask
 *        will be set in the register. Other bits will be ignored.
 */
void set_bits(uint8_t& reg, uint8_t bitmask, uint8_t bits)
{
  reg &= ~bitmask;
  reg |= (bits & bitmask);
}

///////////////////////////////////////////////////////////////////////////////
/// Gripper activation request.
///

constexpr uint8_t rACT_mask = 0b00000001;

void set_gripper_activation(uint8_t& reg, const GripperActivationAction gripper_activation_action)
{
  switch (gripper_activation_action)
  {
    case GripperActivationAction::RESET:
      set_bits(reg, rACT_mask, 0b00000000);
      break;
    case GripperActivationAction::ACTIVE:
      set_bits(reg, rACT_mask, 0b00000001);
      break;
    default:
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
/// Mode change request
///

constexpr uint8_t rMOD_mask = 0b00000110;

void set_grasping_mode(uint8_t& reg, const GraspingMode grasping_mode)
{
  switch (grasping_mode)
  {
    case GraspingMode::BASIC:
      set_bits(reg, rMOD_mask, 0b00000000);
      break;
    case GraspingMode::PINCH:
      set_bits(reg, rMOD_mask, 0b00000010);
      break;
    case GraspingMode::WIDE:
      set_bits(reg, rMOD_mask, 0b00000100);
      break;
    case GraspingMode::SCISSOR:
      set_bits(reg, rMOD_mask, 0b00000110);
      break;
    default:
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
/// GoTo request
///

constexpr uint8_t rGTO_mask = 0b00001000;

void set_go_to(uint8_t& reg, const GoTo go_to)
{
  switch (go_to)
  {
    case GoTo::STOP:
      set_bits(reg, rGTO_mask, 0b00000000);
      break;
    case GoTo::GO_TO:
      set_bits(reg, rGTO_mask, 0b00001000);
      break;
    default:
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
/// Automatic release request
///

constexpr uint8_t rATR_mask = 0b00010000;

void set_automatic_release(uint8_t& reg, const bool automatic_release)
{
  if (automatic_release)
  {
    set_bits(reg, rATR_mask, 0b00010000);
  }
  else
  {
    set_bits(reg, rATR_mask, 0b00000000);
  }
}

///////////////////////////////////////////////////////////////////////////////
/// Individual control mode
///

constexpr uint8_t rICF_mask = 0b00000100;

void set_individual_control_mode(uint8_t& reg, const bool individual_control_mode)
{
  if (individual_control_mode)
  {
    set_bits(reg, rICF_mask, 0b00000100);
  }
  else
  {
    set_bits(reg, rICF_mask, 0b00000000);
  }
}

///////////////////////////////////////////////////////////////////////////////
/// Individual scissor control mode
///

constexpr uint8_t rICS_mask = 0b00001000;

void set_individual_scissor_control_mode(uint8_t& reg, const bool individual_scissor_control_mode)
{
  if (individual_scissor_control_mode)
  {
    set_bits(reg, rICS_mask, 0b00001000);
  }
  else
  {
    set_bits(reg, rICS_mask, 0b00000000);
  }
}

//////////// Getter utils ////////////////////////////////////////////////////

constexpr uint8_t gACT_mask = 0b00000001;

GripperActivationAction get_gripper_activation_action(const uint8_t& reg)
{
  static const std::unordered_map<uint8_t, GripperActivationAction> map{
    { 0b00000000, GripperActivationAction::RESET }, { 0b00000001, GripperActivationAction::ACTIVE }
  };
  return map.at(reg & default_driver_utils::gACT_mask);
}

const std::string gripper_activation_action_to_string(const GripperActivationAction gripper_activation_action)
{
  static std::map<GripperActivationAction, std::string> map = {
    { GripperActivationAction::RESET, "ClearGripperFaultStatus" },
    { GripperActivationAction::ACTIVE, "Activate" },
  };
  return map.at(gripper_activation_action);
}

///////////////////////////////////////////////////////////////////////////////
/// Gripper mode.
///

constexpr uint8_t gMOD_mask = 0b00000110;

GraspingMode get_grasping_mode(const uint8_t& reg)
{
  static const std::unordered_map<uint8_t, GraspingMode> map{ { 0b00000000, GraspingMode::BASIC },
                                                              { 0b00000010, GraspingMode::PINCH },
                                                              { 0b00000100, GraspingMode::WIDE },
                                                              { 0b00000110, GraspingMode::SCISSOR } };
  return map.at(reg & default_driver_utils::gMOD_mask);
}

const std::string grasping_mode_to_string(const GraspingMode grasping_mode)
{
  static std::map<GraspingMode, std::string> map = { { GraspingMode::BASIC, "BASIC" },
                                                     { GraspingMode::PINCH, "PINCH" },
                                                     { GraspingMode::WIDE, "WIDE" },
                                                     { GraspingMode::SCISSOR, "SCISSOR" } };
  return map.at(grasping_mode);
}

///////////////////////////////////////////////////////////////////////////////
/// Regulate.
///

constexpr uint8_t gGTO_mask = 0b00001000;

GoTo get_go_to_status(const uint8_t& reg)
{
  static const std::unordered_map<uint8_t, GoTo> map{ { 0b00000000, GoTo::STOP }, { 0b00001000, GoTo::GO_TO } };
  return map.at(reg & default_driver_utils::gGTO_mask);
}

const std::string go_to_to_string(const GoTo go_to)
{
  static std::map<GoTo, std::string> map = { { GoTo::STOP, "STOP" }, { GoTo::GO_TO, "GO_TO" } };
  return map.at(go_to);
}

///////////////////////////////////////////////////////////////////////////////
/// gripper status
///

constexpr uint8_t gIMC_mask = 0b00110000;

GripperStatus get_gripper_status(const uint8_t& reg)
{
  static const std::unordered_map<uint8_t, GripperStatus> map{ { 0b00000000, GripperStatus::RESET },
                                                               { 0b00010000, GripperStatus::ACTIVATION_IN_PROGRESS },
                                                               { 0b00100000, GripperStatus::MODE_CHANGE_IN_PROGRESS },
                                                               { 0b00110000, GripperStatus::ACTIVATED } };
  return map.at(reg & default_driver_utils::gIMC_mask);
}

///////////////////////////////////////////////////////////////////////////////
/// motion status.
///

constexpr uint8_t gSTA_mask = 0b11000000;

MotionStatus get_motion_status(const uint8_t& reg)
{
  static const std::unordered_map<uint8_t, MotionStatus> map{ { 0b00000000, MotionStatus::IN_MOTION },
                                                              { 0b01000000, MotionStatus::STOPPED_ONE_OR_TWO_UNREACHED },
                                                              { 0b10000000, MotionStatus::STOPPED_THREE_UNREACHED },
                                                              { 0b11000000, MotionStatus::STOPPED_REACHED } };
  return map.at(reg & default_driver_utils::gSTA_mask);
}

const std::string motion_status_to_string(const MotionStatus motionStatus)
{
  static std::map<MotionStatus, std::string> map = {
    { MotionStatus::IN_MOTION, "IN_MOTION" },
    { MotionStatus::STOPPED_ONE_OR_TWO_UNREACHED, "STOPPED_ONE_OR_TWO_UNREACHED" },
    { MotionStatus::STOPPED_THREE_UNREACHED, "STOPPED_THREE_UNREACHED" },
    { MotionStatus::STOPPED_REACHED, "STOPPED_REACHED" }
  };
  return map.at(motionStatus);
}

///////////////////////////////////////////////////////////////////////////////
/// Finger A Object status
///

constexpr uint8_t gDTA_mask = 0b00000011;

ObjectDetectionStatus get_finger_a_object_status(const uint8_t& reg)
{
  static const std::unordered_map<uint8_t, ObjectDetectionStatus> map{
    { 0b00000000, ObjectDetectionStatus::MOVING },
    { 0b00000001, ObjectDetectionStatus::OBJECT_DETECTED_OPENING },
    { 0b00000010, ObjectDetectionStatus::OBJECT_DETECTED_CLOSING },
    { 0b00000011, ObjectDetectionStatus::AT_REQUESTED_POSITION }
  };
  return map.at(reg & default_driver_utils::gDTA_mask);
}

///////////////////////////////////////////////////////////////////////////////
/// Finger B Object status
///

constexpr uint8_t gDTB_mask = 0b00001100;

ObjectDetectionStatus get_finger_b_object_status(const uint8_t& reg)
{
  static const std::unordered_map<uint8_t, ObjectDetectionStatus> map{
    { 0b00000000, ObjectDetectionStatus::MOVING },
    { 0b00000100, ObjectDetectionStatus::OBJECT_DETECTED_OPENING },
    { 0b00001000, ObjectDetectionStatus::OBJECT_DETECTED_CLOSING },
    { 0b00001100, ObjectDetectionStatus::AT_REQUESTED_POSITION }
  };
  return map.at(reg & default_driver_utils::gDTB_mask);
}

///////////////////////////////////////////////////////////////////////////////
/// Finger C Object status
///

constexpr uint8_t gDTC_mask = 0b00110000;

ObjectDetectionStatus get_finger_c_object_status(const uint8_t& reg)
{
  static const std::unordered_map<uint8_t, ObjectDetectionStatus> map{
    { 0b00000000, ObjectDetectionStatus::MOVING },
    { 0b00010000, ObjectDetectionStatus::OBJECT_DETECTED_OPENING },
    { 0b00100000, ObjectDetectionStatus::OBJECT_DETECTED_CLOSING },
    { 0b00110000, ObjectDetectionStatus::AT_REQUESTED_POSITION }
  };
  return map.at(reg & default_driver_utils::gDTC_mask);
}

///////////////////////////////////////////////////////////////////////////////
/// Scissor Object status
///

constexpr uint8_t gDTS_mask = 0b11000000;

ObjectDetectionStatus get_scissor_object_status(const uint8_t& reg)
{
  static const std::unordered_map<uint8_t, ObjectDetectionStatus> map{
    { 0b00000000, ObjectDetectionStatus::MOVING },
    { 0b01000000, ObjectDetectionStatus::OBJECT_DETECTED_OPENING },
    { 0b10000000, ObjectDetectionStatus::OBJECT_DETECTED_CLOSING },
    { 0b11000000, ObjectDetectionStatus::AT_REQUESTED_POSITION }
  };
  return map.at(reg & default_driver_utils::gDTS_mask);
}

///////////////////////////////////////////////////////////////////////////////
/// Fault status
///

constexpr uint8_t gFLT_mask = 0b00001111;

GripperFaultStatus get_gripper_fault_status(const uint8_t& reg)
{
  static const std::unordered_map<uint8_t, GripperFaultStatus> map{
    { 0b00000000, GripperFaultStatus::ACTION_DELAYED_MODE_CHANGE_NEEDED },
    { 0b00000001, GripperFaultStatus::ACTION_DELAYED_REACTIVATION_NEEDED },
    { 0b00000010, GripperFaultStatus::ACTIVATION_BIT_NOT_SET },
    { 0b00000011, GripperFaultStatus::NOT_READY },
    { 0b00000100, GripperFaultStatus::MODE_CHANGE_SCISSOR_MOTION_ERROR },
    { 0b00000101, GripperFaultStatus::AUTO_RELEASE_IN_PROGRESS },
    { 0b00000110, GripperFaultStatus::ACTIVATION_FAULT },
    { 0b00000111, GripperFaultStatus::MODE_CHANGE_FAULT },
    { 0b00001000, GripperFaultStatus::AUTOMATIC_RELEASE_FAULT },
  };
  return map.at(reg & default_driver_utils::gFLT_mask);
}

const std::string fault_status_to_string(const GripperFaultStatus fault_status)
{
  static std::map<GripperFaultStatus, std::string> map = {
    { GripperFaultStatus::ACTION_DELAYED_REACTIVATION_NEEDED, "ACTION_DELAYED_REACTIVATION_NEEDED" },
    { GripperFaultStatus::ACTION_DELAYED_MODE_CHANGE_NEEDED, "ACTION_DELAYED_MODE_CHANGE_NEEDED" },
    { GripperFaultStatus::ACTIVATION_BIT_NOT_SET, "ACTIVATION_BIT_NOT_SET" },
    { GripperFaultStatus::NOT_READY, "NOT_READY" },
    { GripperFaultStatus::MODE_CHANGE_SCISSOR_MOTION_ERROR, "MODE_CHANGE_SCISSOR_MOTION_ERROR" },
    { GripperFaultStatus::AUTO_RELEASE_IN_PROGRESS, "AUTO_RELEASE_IN_PROGRESS" },
    { GripperFaultStatus::ACTIVATION_FAULT, "ACTIVATION_FAULT" },
    { GripperFaultStatus::MODE_CHANGE_FAULT, "MODE_CHANGE_FAULT" },
    { GripperFaultStatus::AUTOMATIC_RELEASE_FAULT, "AUTOMATIC_RELEASE_FAULT" },
  };
  return map.at(fault_status);
}

}  // namespace robotiq_3f_driver::default_driver_utils
