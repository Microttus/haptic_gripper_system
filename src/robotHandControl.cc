//
// Created by Martin Økter on 15/10/2023.
//

#include <iostream>
#include <unistd.h>


#include "qbshr_ctr/qbSoftHandHandler.hh"
#include "qbshr_ctr/qbSoftHandControl.hh"


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"



qbSoftHandHandler my_hands_;

int main() {

    std::vector<qbSoftHandControl> qbSoftHand_devices = my_hands_.ReturnDeviceMap();
    std::cout << "There are " << qbSoftHand_devices.size() << " available qbSoftHand Research available for control" << std::endl;
    sleep(2);
    std::cout << "Start movement" << std::endl;
    qbSoftHand_devices[0].SetMotorStates(true);
    sleep(2);
    qbSoftHand_devices[0].SetGripValue(0,10000);
    sleep(2);
    qbSoftHand_devices[0].SetGripValue(0,0);
    qbSoftHand_devices[0].GetCurrents();
    qbSoftHand_devices[0].GetPositions();
    sleep(2);

    std::cout << "Main function completed" << std::endl;
};