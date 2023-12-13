#include <robotiq_3f_driver/default_driver.hpp>
#include <robotiq_3f_driver/default_serial_factory.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace robotiq_3f_driver;

int main()
{
  // set the log level to DEBUG
  rcutils_logging_set_logger_level("DefaultDriver", RCUTILS_LOG_SEVERITY_DEBUG);
  rcutils_logging_set_logger_level("DefaultSerialFactory", RCUTILS_LOG_SEVERITY_DEBUG);

  hardware_interface::HardwareInfo info{
    .name = "robotiq_3f_gripper",
    .type = "system",
    .hardware_class_type = "robotiq_3f_driver/Robotiq3fGripperHardwareInterface",
  };
  info.hardware_parameters["usb_port"] = "/dev/ttyUSB1";

  // Create a serial interface
  auto serial = DefaultSerialFactory().create(info);

  // Create the driver
  DefaultDriver driver(std::move(serial));

  driver.set_slave_address(0x09);

  driver.connect();

  driver.activate();

  sleep(1);
}