//
// Created by Martin Økter on 24/09/2023.
//

#include <memory>
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SimpleThimbleControl : public rclcpp::Node
{
public:
    SimpleThimbleControl() : Node("simple_thimble_control") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
                "test_topic", 10, std::bind(&SimpleThimbleControl::topic_callback, this, _1));

        int serial_port = open("/dev/ttyUSB0", O_RDWR);

        // Check for errors
        if (serial_port < 0) {
            //printf("Error %i from open: %s\n", errno, strerror(errno));
            RCLCPP_INFO(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
        }

        if(tcgetattr(serial_port, &tty) != 0) {
            //printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            RCLCPP_INFO(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
        }
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    struct termios tty;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleThimbleControl>());
    rclcpp::shutdown();
    return 0;
}