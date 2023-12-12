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

#include <string>

#include <robotiq_3f_driver/driver.hpp>

/**
 * In this utility class we have methods to:
 * - set and get information in and out of a byte register using enums;
 * - converts enums into strings for testing and debugging.
 */
namespace robotiq_3f_driver::default_driver_utils
{

/** These represent read and write actions. */
enum class FunctionCode : uint8_t
{
  ReadInputRegisters = 0x03,
  PresetSingleRegister = 0x06,
  PresetMultipleRegisters = 0x10, // 16 in decimal
  MasterReadWriteMultipleRegisters = 0x17, // 23 in decimal
};

// Setters
void set_gripper_activation(uint8_t& reg, const GripperActivationAction gripper_activation_action);
void set_grasping_mode(uint8_t& reg, const GraspingMode grasping_mode);
void set_go_to(uint8_t& reg, const GoTo go_to);
void set_automatic_release(uint8_t& reg, const bool automatic_release);
void set_individual_control_mode(uint8_t& reg, const bool individual_control_mode);
void set_individual_scissor_control_mode(uint8_t& reg, const bool individual_scissor_control_mode);

// getters
GripperActivationAction get_gripper_activation_action(const uint8_t& reg);
const std::string gripper_activation_action_to_string(const GripperActivationAction gripper_activation_action);

GraspingMode get_grasping_mode(const uint8_t& reg);
const std::string grasping_mode_to_string(const GraspingMode grasping_mode);

GoTo get_go_to_status(const uint8_t& reg);
const std::string go_to_to_string(const GoTo go_to);

GripperStatus get_gripper_status(const uint8_t& reg);

MotionStatus get_motion_status(const uint8_t& reg);
const std::string motion_status_to_string(const MotionStatus motionStatus);

ObjectDetectionStatus get_finger_a_object_status(const uint8_t& reg);
ObjectDetectionStatus get_finger_b_object_status(const uint8_t& reg);
ObjectDetectionStatus get_finger_c_object_status(const uint8_t& reg);
ObjectDetectionStatus get_scissor_object_status(const uint8_t& reg);

GripperFaultStatus get_gripper_fault_status(const uint8_t& reg);
const std::string fault_status_to_string(const GripperFaultStatus fault_status);

};  // namespace robotiq_3f_driver::default_driver_utils
