/*
 * Created by Martin Økter
 *
 * created on 23/09/2023
 *
 * A node for connecting the control of a robotic hand
 * and the force feedback to a operator hand
 *
 * Future versions will incude arm tracking and robotic
 * arm control
 */


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

#include "qbshr_ctr/qbSoftHandHandler.hh"
#include "qbshr_ctr/qbSoftHandControl.hh"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;


struct FingerForceStruct {
    int thumb;
    int index;
    int middle;
    int ring;
    int little;
    int palm;
};

struct RobotHandFeedback {
  std::vector<int16_t> positions;
  std::vector<int16_t> currents;
  int force_compensated;
};

class HandInterface : public rclcpp::Node
{
public:
    HandInterface() : Node("hand_control")
    , qbSoftHand_devices(my_hands_.ReturnDeviceMap())
    , temp_grip_state(0)
    {
      heartbeat_ = this->create_publisher<std_msgs::msg::String>("heartbeat_control", 10);        // Heartbeat
      right_hand_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("finger_force", 10);    // Finger force publisher


      timer_ = this->create_wall_timer(10ms, std::bind(&HandInterface::timer_callback, this));

      // Initialization of values
      RightHand.thumb = 0;
      RightHand.index = 0;

      // Setup of the robotic hand
      sleep(2);
      std::cout << "There are " << qbSoftHand_devices.size() << " available qbSoftHand Research available for control" << std::endl;
      sleep(2);
      qbSoftHand_devices[0].SetMotorStates(true);
      sleep(2);


      std::cout << "Setup completed" << std::endl;
    }

private:
  void timer_callback()
  {
    heartbeat_update();
    set_robot_hand_pos();
    update_robot_hand_feedback();
    update_finger_force();
    apply_finger_force();

  }

  void heartbeat_update()
  {
    auto message = std_msgs::msg::String();
    message.data = std::to_string(force_from_robot_hand);
    //message.header.stamp = this->get_clock()->now();
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    heartbeat_->publish(message);
  }

  void apply_finger_force()
  {
    auto finger_msg = geometry_msgs::msg::Twist();
    finger_msg.linear.x = RightHand.thumb;
    finger_msg.linear.y = RightHand.index;
    finger_msg.linear.z = RightHand.middle;
    finger_msg.angular.x = RightHand.ring;
    finger_msg.angular.y = RightHand.little;
    finger_msg.angular.z = RightHand.palm;
    right_hand_pub_->publish(finger_msg);
  }

  void update_finger_force()
  {
    force_from_robot_hand = RightRobotHand.force_compensated;

    //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    force_from_robot_hand = (force_from_robot_hand - 0) * (254 - 0) / (400 - 0) + 0;

    if (force_from_robot_hand > 254) {
      force_from_robot_hand = 254;
    } else if (force_from_robot_hand < 0) {
      force_from_robot_hand = 0;
    }

    RightHand.thumb = force_from_robot_hand;
    RightHand.index = force_from_robot_hand;
  }

  void set_robot_hand_pos()
  {
    temp_grip_state+=10;

    if (temp_grip_state > 18000){
      temp_grip_state = 0;
    }

    qbSoftHand_devices[0].SetGripValue(temp_grip_state,0);
  }

  void update_robot_hand_feedback()
  {
    RightRobotHand.positions = qbSoftHand_devices[0].GetPositions();
    RightRobotHand.currents = qbSoftHand_devices[0].GetCurrents();

    RightRobotHand.force_compensated = RightRobotHand.currents.at(0) - (RightRobotHand.positions.at(0)*0.03);
  }

  // ROS2 and publishers declarations
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr right_hand_pub_;

  // Finger force data struct declaration
  FingerForceStruct RightHand;

  // Robotic hand object
  qbSoftHandHandler my_hands_;
  std::vector<qbSoftHandControl> qbSoftHand_devices;
  RobotHandFeedback RightRobotHand;

  // Temp test values // TODO: Remove during cleanup
  int temp_fing_force_ = 180;
  int temp_grip_state;
  int force_from_robot_hand = 0;
  //size_t count_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandInterface>());
  rclcpp::shutdown();
  return 0;
}
