// Copyright 2023, Peter Mitrano

#pragma once

#include <memory>

#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_loader.hpp>

#include <robotiq_3f_transmission_plugins/individual_control_transmission.hpp>

namespace robotiq_3f_transmission_plugins
{

class IndividualControlTransmissionLoader : public transmission_interface::TransmissionLoader
{
public:
  std::shared_ptr<transmission_interface::Transmission>
  load(const hardware_interface::TransmissionInfo& transmission_info) override;
};

}  // namespace robotiq_3f_transmission_plugins