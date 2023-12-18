#include <robotiq_3f_driver/default_driver.hpp>
#include <robotiq_3f_driver/default_serial.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <robotiq_3f_interfaces/srv/change_grasping_mode.hpp>
#include <robotiq_3f_interfaces/msg/status.hpp>
#include <robotiq_3f_interfaces/msg/simple_control_command.hpp>
#include <robotiq_3f_interfaces/msg/independent_control_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robotiq_3f_transmission_plugins/individual_control_transmission.hpp>

using namespace std::chrono_literals;

using namespace robotiq_3f_driver;

auto mode_msg_to_mode(robotiq_3f_interfaces::msg::GraspingMode mode_msg)
{
  switch (mode_msg.mode)
  {
    case robotiq_3f_interfaces::msg::GraspingMode::BASIC:
      return GraspingMode::BASIC;
    case robotiq_3f_interfaces::msg::GraspingMode::PINCH:
      return GraspingMode::PINCH;
    case robotiq_3f_interfaces::msg::GraspingMode::WIDE:
      return GraspingMode::WIDE;
    case robotiq_3f_interfaces::msg::GraspingMode::SCISSOR:
      return GraspingMode::SCISSOR;
    default:
      throw std::runtime_error("Unknown grasping mode");
  }
}

auto mode_to_mode_msg(GraspingMode mode)
{
  switch (mode)
  {
    case GraspingMode::BASIC:
      return robotiq_3f_interfaces::msg::GraspingMode::BASIC;
    case GraspingMode::PINCH:
      return robotiq_3f_interfaces::msg::GraspingMode::PINCH;
    case GraspingMode::WIDE:
      return robotiq_3f_interfaces::msg::GraspingMode::WIDE;
    case GraspingMode::SCISSOR:
      return robotiq_3f_interfaces::msg::GraspingMode::SCISSOR;
    default:
      throw std::runtime_error("Unknown grasping mode");
  }
}


auto obj_to_obj_msg(ObjectDetectionStatus obj)
{
  switch (obj)
  {
    case ObjectDetectionStatus::MOVING:
      return robotiq_3f_interfaces::msg::ObjectDetectionStatus::MOVING;
    case ObjectDetectionStatus::OBJECT_DETECTED_OPENING:
      return robotiq_3f_interfaces::msg::ObjectDetectionStatus::OBJECT_DETECTED_OPENING;
    case ObjectDetectionStatus::OBJECT_DETECTED_CLOSING:
      return robotiq_3f_interfaces::msg::ObjectDetectionStatus::OBJECT_DETECTED_CLOSING;
    case ObjectDetectionStatus::AT_REQUESTED_POSITION:
      return robotiq_3f_interfaces::msg::ObjectDetectionStatus::AT_REQUESTED_POSITION;
    default:
      throw std::runtime_error("Unknown object detection status");
  }
}

class ROSDriver : public rclcpp::Node
{
public:
  explicit ROSDriver() : Node("robotiq_3f_driver")
  {
    auto serial = std::make_unique<DefaultSerial>();
    serial->set_port("/dev/ttyUSB1");
    serial->set_baudrate(115200);
    serial->set_timeout(500ms);

    driver_ = std::make_unique<DefaultDriver>(std::move(serial));
    driver_->set_slave_address(0x09);

    driver_->connect();
    driver_->clear_faults();
    driver_->activate();

    // Set up the ROS API
    set_grasping_mode_service_ = create_service<robotiq_3f_interfaces::srv::ChangeGraspingMode>(
        "change_grasping_mode", [&](const std::shared_ptr<robotiq_3f_interfaces::srv::ChangeGraspingMode::Request> req,
                                    std::shared_ptr<robotiq_3f_interfaces::srv::ChangeGraspingMode::Response> res) {
          latest_mode_ = mode_msg_to_mode(req->mode);
          driver_->send_simple_control_command(latest_mode_, 0, 0, 0);

          // Wait until the motion status said we've finished changing modes.
          // and yes it really can take a long time!
          auto const success = driver_->wait_until_reached(30.0);

          res->success = success;
        });

    status_pub_ = create_publisher<robotiq_3f_interfaces::msg::Status>("status", 10);
    rclcpp::QoS js_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", js_qos);

    independent_control_command_sub_ = create_subscription<robotiq_3f_interfaces::msg::IndependentControlCommand>(
        "independent_control_command", 10,
        [&](const std::shared_ptr<robotiq_3f_interfaces::msg::IndependentControlCommand> msg) {
          IndependentControlCommand cmd;
          cmd.finger_a_position = msg->finger_a_position;
          cmd.finger_b_position = msg->finger_b_position;
          cmd.finger_c_position = msg->finger_c_position;
          cmd.scissor_position = msg->scissor_position;
          cmd.finger_a_velocity = msg->finger_a_velocity;
          cmd.finger_b_velocity = msg->finger_b_velocity;
          cmd.finger_c_velocity = msg->finger_c_velocity;
          cmd.scissor_velocity = msg->scissor_velocity;
          cmd.finger_a_force = msg->finger_a_force;
          cmd.finger_b_force = msg->finger_b_force;
          cmd.finger_c_force = msg->finger_c_force;
          cmd.scissor_force = msg->scissor_force;

          driver_->send_independent_control_command(cmd);
        });

    simple_control_command_sub_ = create_subscription<robotiq_3f_interfaces::msg::SimpleControlCommand>(
        "simple_control_command", 10, [&](const std::shared_ptr<robotiq_3f_interfaces::msg::SimpleControlCommand> msg) {
          driver_->send_simple_control_command(latest_mode_, msg->position, msg->velocity, msg->force);
        });

    // Create a timer callback at 100HZ to publish the states
    timer_ = create_wall_timer(10ms, [&]() {
      auto status = driver_->get_full_status();

      robotiq_3f_interfaces::msg::Status status_msg;
      status_msg.mode.mode = mode_to_mode_msg(latest_mode_);
      status_msg.finger_a_position = status.finger_a_position;
      status_msg.finger_b_position = status.finger_b_position;
      status_msg.finger_c_position = status.finger_c_position;
      status_msg.scissor_position = status.scissor_position;
      status_msg.finger_a_current = status.finger_a_current;
      status_msg.finger_b_current = status.finger_b_current;
      status_msg.finger_c_current = status.finger_c_current;
      status_msg.scissor_current = status.scissor_current;
      status_msg.finger_a_cmd_echo = status.finger_a_position_cmd_echo;
      status_msg.finger_b_cmd_echo = status.finger_b_position_cmd_echo;
      status_msg.finger_c_cmd_echo = status.finger_c_position_cmd_echo;
      status_msg.scissor_cmd_echo = status.scissor_position_cmd_echo;
      status_msg.finger_a_object_detection.status = obj_to_obj_msg(status.finger_a_object_detection_status);
      status_msg.finger_b_object_detection.status = obj_to_obj_msg(status.finger_b_object_detection_status);
      status_msg.finger_c_object_detection.status = obj_to_obj_msg(status.finger_c_object_detection_status);
      status_msg.scissor_object_detection.status = obj_to_obj_msg(status.scissor_object_detection_status);

      status_pub_->publish(status_msg);

      auto const finger_a_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(status.finger_a_position);
      auto const finger_b_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(status.finger_b_position);
      auto const finger_c_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(status.finger_c_position);
      auto const palm_finger = robotiq_3f_transmission_plugins::get_palm_finger_pos(status.scissor_position);

      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = now();
      joint_state_msg.name = { "finger_a_joint_1", "finger_a_joint_2",    "finger_a_joint_3",   "finger_b_joint_1",
                               "finger_b_joint_2", "finger_b_joint_3",    "finger_c_joint_1",   "finger_c_joint_2",
                               "finger_c_joint_3", "palm_finger_b_joint", "palm_finger_c_joint" };
      joint_state_msg.position = { finger_a_thetas[0], finger_a_thetas[1], finger_a_thetas[2], finger_b_thetas[0],
                                   finger_b_thetas[1], finger_b_thetas[2], finger_c_thetas[0], finger_c_thetas[1],
                                   finger_c_thetas[2], palm_finger,       -palm_finger };

      joint_state_pub_->publish(joint_state_msg);
    });
  }

  ~ROSDriver()
  {
    driver_->deactivate();
    driver_->disconnect();
  }

private:
  std::unique_ptr<DefaultDriver> driver_;

  rclcpp::Service<robotiq_3f_interfaces::srv::ChangeGraspingMode>::SharedPtr set_grasping_mode_service_;
  rclcpp::Publisher<robotiq_3f_interfaces::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<robotiq_3f_interfaces::msg::IndependentControlCommand>::SharedPtr independent_control_command_sub_;
  rclcpp::Subscription<robotiq_3f_interfaces::msg::SimpleControlCommand>::SharedPtr simple_control_command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  GraspingMode latest_mode_ = GraspingMode::BASIC;
};

int main(int argc, const char** argv)
{
  // Uncomment to set the log level to DEBUG
  //  rcutils_logging_set_logger_level("DefaultDriver", RCUTILS_LOG_SEVERITY_DEBUG);
  //  rcutils_logging_set_logger_level("DefaultSerialFactory", RCUTILS_LOG_SEVERITY_DEBUG);

  // Initialize ROS
  rclcpp::init(argc, argv);

  // Instantiate the ROS Wrapper
  ROSDriver ros_driver;

  rclcpp::spin(ros_driver.get_node_base_interface());

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
  //  // we want to close finger c slowly
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
  //
  //  sleep(1);
  //
  //  driver.send_simple_control_command(GraspingMode::BASIC, 1.0, 1.0, 1.0);
  //  std::cout << driver.wait_until_reached(5.0) << std::endl;
  //
  //  driver.send_simple_control_command(GraspingMode::BASIC, 0.0, 1.0, 1.0);
  //  std::cout << driver.wait_until_reached(5.0) << std::endl;
  //
  //  driver.send_simple_control_command(GraspingMode::BASIC, 1.0, 0.4, 0.1);
  //  std::cout << driver.wait_until_reached(5.0) << std::endl;
  //
  //  driver.send_simple_control_command(GraspingMode::BASIC, 0.0, 1.0, 0.1);
  //  std::cout << driver.wait_until_reached(5.0) << std::endl;
  //
  //  sleep(3);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Done!");
}