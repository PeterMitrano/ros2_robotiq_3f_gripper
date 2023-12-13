#include <robotiq_3f_driver/default_driver.hpp>
#include <robotiq_3f_driver/default_serial_factory.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace robotiq_3f_driver;

int main()
{
  // set the log level to DEBUG
//  rcutils_logging_set_logger_level("DefaultDriver", RCUTILS_LOG_SEVERITY_DEBUG);
//  rcutils_logging_set_logger_level("DefaultSerialFactory", RCUTILS_LOG_SEVERITY_DEBUG);

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

//  IndependentControlCommand cmd;
//  cmd.finger_a_position = 0.0;
//  cmd.finger_b_position = 0.0;
//  cmd.finger_c_position = 0.0;
//  cmd.scissor_position = 0.0;
//  cmd.finger_a_velocity = 0.0;
//  cmd.finger_b_velocity = 0.0;
//  cmd.finger_c_velocity = 1.0;
//  cmd.scissor_velocity = 0.0;
//  cmd.finger_a_force = 0;
//  cmd.finger_b_force = 0;
//  cmd.finger_c_force = 0.5;
//  cmd.scissor_force = 0;
//
//  driver.send_independent_control_command(cmd);
//
//  // we want to close finger C slowly
//  for (int i = 0; i < 10; ++i)
//  {
//    cmd.finger_c_position += 0.1;
//    driver.send_independent_control_command(cmd);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//  }
//
//  // then open again
//  for (int i = 0; i < 10; ++i)
//  {
//    cmd.finger_c_position -= 0.1;
//    driver.send_independent_control_command(cmd);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//  }

  sleep(1);

  driver.send_simple_control_command(GraspingMode::BASIC, 0.5, 1.0, 1.0);
  driver.wait_until_reached(1.0);

  driver.send_simple_control_command(GraspingMode::PINCH, 0.5, 1.0, 1.0);
  driver.wait_until_reached(1.0);

  driver.send_simple_control_command(GraspingMode::WIDE, 0.5, 1.0, 1.0);
  driver.wait_until_reached(1.0);

  sleep(3);

  driver.deactivate();

  driver.disconnect();

  RCLCPP_INFO(rclcpp::get_logger("main"), "Done!");
}