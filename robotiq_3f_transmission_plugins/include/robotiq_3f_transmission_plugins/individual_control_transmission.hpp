// Copyright 2023, Peter Mitrano.

#pragma once

#include <memory>

#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_loader.hpp>

namespace robotiq_3f_transmission_plugins
{

std::array<double, 3> get_finger_thetas(double g);
double get_palm_finger_pos(double scissor_pos);

class IndividualControlTransmission : public transmission_interface::Transmission
{
public:
  IndividualControlTransmission() = default;

  ~IndividualControlTransmission() override = default;

  void configure(const std::vector<transmission_interface::JointHandle>& joint_handles,
                 const std::vector<transmission_interface::ActuatorHandle>& actuator_handles) override;

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

  std::unique_ptr<transmission_interface::JointHandle> finger_c_joint_1_;
  std::unique_ptr<transmission_interface::JointHandle> finger_c_joint_2_;
  std::unique_ptr<transmission_interface::JointHandle> finger_c_joint_3_;
  std::unique_ptr<transmission_interface::JointHandle> finger_b_joint_1_;
  std::unique_ptr<transmission_interface::JointHandle> finger_b_joint_2_;
  std::unique_ptr<transmission_interface::JointHandle> finger_b_joint_3_;
  std::unique_ptr<transmission_interface::JointHandle> finger_a_joint_1_;
  std::unique_ptr<transmission_interface::JointHandle> finger_a_joint_2_;
  std::unique_ptr<transmission_interface::JointHandle> finger_a_joint_3_;
  std::unique_ptr<transmission_interface::JointHandle> palm_finger_c_joint_;
  std::unique_ptr<transmission_interface::JointHandle> palm_finger_b_joint_;

  double get_palm_finger_pos(const double scissor_pos) const;
};
}  // namespace robotiq_3f_transmission_plugins