// Copyright 2023, Peter Mitrano

#include <robotiq_3f_transmission_plugins/individual_control_transmission_loader.hpp>
#include <robotiq_3f_transmission_plugins/individual_control_transmission.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace robotiq_3f_transmission_plugins
{

std::shared_ptr<transmission_interface::Transmission>
IndividualControlTransmissionLoader::load(const hardware_interface::TransmissionInfo& transmission_info)
{
  return std::shared_ptr<transmission_interface::Transmission>(new IndividualControlTransmission());
}
}  // namespace robotiq_3f_transmission_plugins

PLUGINLIB_EXPORT_CLASS(robotiq_3f_transmission_plugins::IndividualControlTransmissionLoader,
                       transmission_interface::TransmissionLoader)