//
// Created by Martin Økter on 23/09/2023.
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "../include/qbSoftHandControl.hh"
#include "../include/qbSoftHandHandler.hh"

using namespace std::chrono_literals;



class HandInterface : public rclcpp::Node
{
public:
    HandInterface()
            : Node("hand_control")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", 10); // CHANGE
        timer_ = this->create_wall_timer(50ms, std::bind(&HandInterface::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = {"base_to_crane_boom"};
        //message.header.stamp = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);

    }
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandInterface>());
    rclcpp::shutdown();
    return 0;
}
