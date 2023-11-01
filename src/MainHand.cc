//
// Created by Martin Økter on 23/09/2023.
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>

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

class HandInterface : public rclcpp::Node
{
public:
    HandInterface() : Node("hand_control")
    {
        heartbeat_ = this->create_publisher<std_msgs::msg::String>("heartbeat_control", 10);        // Heartbeat
        right_hand_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("finger_force", 10);    // Finger force publisher



        timer_ = this->create_wall_timer(50ms, std::bind(&HandInterface::timer_callback, this));

        rightHand.thumb = 0;

    }

private:
  void timer_callback()
  {
      //heartbeat_update();
      update_finger_force();
      apply_finger_force();
  }

  void heartbeat_update()
  {
    auto message = std_msgs::msg::String();
    message.data = {"base_to_crane_boom"};
    //message.header.stamp = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    heartbeat_->publish(message);
  }

  void apply_finger_force()
  {
    auto finger_msg = geometry_msgs::msg::Twist();
    finger_msg.linear.x = rightHand.thumb;
    finger_msg.linear.y = rightHand.index;
    finger_msg.linear.z = rightHand.middle;
    finger_msg.angular.x = rightHand.ring;
    finger_msg.angular.y = rightHand.little;
    finger_msg.angular.z = rightHand.palm;
    right_hand_pub_->publish(finger_msg);
  }

  void update_finger_force()
  {
      temp_fing_force_++;
      if (temp_fing_force_ > 230) {
        temp_fing_force_ = 180;
      }

      rightHand.thumb = temp_fing_force_;
      rightHand.index = temp_fing_force_;
  }



  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr right_hand_pub_;

  FingerForceStruct rightHand;

  int temp_fing_force_ = 180;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandInterface>());
  rclcpp::shutdown();
  return 0;
}
