// Copyright (c) 2023 PickNik, Inc.
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

#include "epick_driver/driver.hpp"

/**
 * In this utility class we have methods to:
 * - set and get information in and out of a byte register using enums;
 * - converts enums into strings for testing and debugging.
 */
namespace epick_driver::driver_utils
{

/** These represent read and write actions. */
enum class FunctionCode : uint8_t
{
  ReadInputRegisters = 0x04,
  PresetSingleRegister = 0x06,
  PresetMultipleRegisters = 0x10,
  MasterReadWriteMultipleRegisters = 0x17,
};

///////////////////////////////////////////////////////////////////////////////
/// Gripper activation request.
///

void set_gripper_activation_action(uint8_t& reg, const GripperActivationAction gripper_activation_action);

GripperActivationAction get_gripper_activation_action(uint8_t& reg);

/**
 * Convert a GripperActivationAction enum into a string.
 * @param gripper_activation_action The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_activation_action_to_string(const GripperActivationAction gripper_activation_action);

///////////////////////////////////////////////////////////////////////////////
/// Gripper mode.
///

void set_gripper_mode(uint8_t& reg, const GripperMode gripper_mode);

GripperMode get_gripper_mode(uint8_t& reg);

/**
 * Convert a GripperMode enum into a string.
 * @param gripper_mode The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_mode_to_string(const GripperMode gripper_mode);

///////////////////////////////////////////////////////////////////////////////
/// Regulate.
///

void set_gripper_regulate_action(uint8_t& reg, const GripperRegulateAction regulate_action);

GripperRegulateAction get_gripper_regulate_action(uint8_t& reg);

/**
 * Convert a GripperRegulateAction enum into a string.
 * @param gripper_regulate_action The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_regulate_action_to_string(const GripperRegulateAction gripper_regulate_action);

///////////////////////////////////////////////////////////////////////////////
/// Automatic release action.
///

/**
 * Convert a GripperReleaseAction enum into a string.
 * @param gripper_release_action The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_release_action_to_string(const GripperReleaseAction gripper_release_action);

///////////////////////////////////////////////////////////////////////////////
/// Activation status.
///

GripperActivationStatus get_gripper_activation_status(uint8_t& reg);

/**
 * Convert a GripperActivationStatus enum into a string.
 * @param gripper_activation_status The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_activation_status_to_string(const GripperActivationStatus gripper_activation_status);

///////////////////////////////////////////////////////////////////////////////
/// Object status
///

ObjectDetectionStatus get_object_detection_status(uint8_t& reg);

/**
 * Convert an ObjectDetection enum into a string.
 * @param object_detection The enum.
 * @return A string representation of the given enum
 */
const std::string object_detection_to_string(const ObjectDetectionStatus object_detection);

///////////////////////////////////////////////////////////////////////////////
/// Gripper fault status
///

GripperFaultStatus get_gripper_fault_status(uint8_t& reg);

/**
 * Convert a FaultStatus enum into a string.
 * @param fault_status The enum.
 * @return A string representation of the given enum
 */
const std::string fault_status_to_string(const GripperFaultStatus fault_status);

///////////////////////////////////////////////////////////////////////////////
/// Actuator status
///

ActuatorStatus get_actuator_status(uint8_t& reg);

/**
 * Convert a ActuatorStatus enum into a string.
 * @param actuator_status The enum.
 * @return A string representation of the given enum
 */
const std::string actuator_status_to_string(const ActuatorStatus actuator_status);

};  // namespace epick_driver::driver_utils
