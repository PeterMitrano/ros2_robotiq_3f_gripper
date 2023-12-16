// Copyright 2023, Peter Mitrano.

#pragma once

#include <memory>

#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_loader.hpp>

namespace robotiq_3f_transmission_plugins
{

class IndividualControlTransmission : public transmission_interface::Transmission
{
public:
  IndividualControlTransmission() = default;
  ~IndividualControlTransmission() override = default;

  void configure(
      const std::vector<transmission_interface::JointHandle> &joint_handles,
      const std::vector<transmission_interface::ActuatorHandle> &actuator_handles) override;

  void actuator_to_joint() override;

  void joint_to_actuator() override;

  std::size_t num_actuators() const override;

  std::size_t num_joints() const override;

private:
  // FIXME: I don't like that we are hard-coding this both here AND in the xml.
  //  can we only do it here?
  std::unique_ptr<transmission_interface::ActuatorHandle> finger_a_actuator_;
  std::unique_ptr<transmission_interface::ActuatorHandle> finger_b_actuator_;
  std::unique_ptr<transmission_interface::ActuatorHandle> finger_c_actuator_;
  std::unique_ptr<transmission_interface::ActuatorHandle> scissor_actuator_;

  std::unique_ptr<transmission_interface::JointHandle> finger_1_joint_1_;
  std::unique_ptr<transmission_interface::JointHandle> finger_1_joint_2_;
  std::unique_ptr<transmission_interface::JointHandle> finger_1_joint_3_;
  std::unique_ptr<transmission_interface::JointHandle> finger_2_joint_1_;
  std::unique_ptr<transmission_interface::JointHandle> finger_2_joint_2_;
  std::unique_ptr<transmission_interface::JointHandle> finger_2_joint_3_;
  std::unique_ptr<transmission_interface::JointHandle> finger_3_joint_1_;
  std::unique_ptr<transmission_interface::JointHandle> finger_3_joint_2_;
  std::unique_ptr<transmission_interface::JointHandle> finger_3_joint_3_;
  std::unique_ptr<transmission_interface::JointHandle> palm_finger_1_joint_;
  std::unique_ptr<transmission_interface::JointHandle> palm_finger_2_joint_;

};
}  // namespace robotiq_3f_transmission_plugins