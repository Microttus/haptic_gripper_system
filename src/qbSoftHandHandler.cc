//
// Created by Martin Økter on 07/10/2023.
//

#include "../include/qbSoftHandHandler.hh"

#include "../lib/qbdevice-api-7.x.x/serial/include/serial/serial.h"
#include "../lib/qbdevice-api-7.x.x/qbrobotics-driver/libs/research/include/qbrobotics_research_api/qbsofthand_research_api.h"

#include <iostream>
#include <string>
#include <iomanip>
#include <memory>
#include <set>
#include <regex>


qbSoftHandHandler::qbSoftHandHandler() {

}

qbSoftHandHandler::~qbSoftHandHandler() {

}

int qbSoftHandHandler::ScanForDevices(const int &max_repeats) {

    communication_handler_ = std::make_shared<qbrobotics_research_api::CommunicationLegacy>(); // make shared pointer that handles the communication

    if (communication_handler_->listSerialPorts(serial_ports_) < 0) {
        std::cerr << "[scanForDevices] no serial ports found" << std::endl;
        return -1;
    }

    int qbrobotics_devices_found = 0;
    for(auto &serial_port:serial_ports_){ // scan and open all the serial port
        int failures = 0;
        while (failures <= max_repeats) {
            if (qbSoftHandHandler::OpenSerialPort(serial_port.serial_port) != 0) {
                failures++;
                continue;
            }
            break;
        }
        if (failures >= max_repeats) {
            continue;
        }

        if (communication_handler_->listConnectedDevices(serial_port.serial_port, device_ids_) >= 0) { // retrieved at least a qbrobotics device
            for(auto &device_id:device_ids_) {
                if (device_id.id == 120 || device_id.id == 0) {
                    std::cout << "Not valid device retrieved!" << std::endl;
                    continue;  // ID 120 is reserved, ID 0 is for sure an error
                }
                soft_hands_.insert(std::make_pair(static_cast<int>(device_id.id), std::make_shared<qbrobotics_research_api::qbSoftHandLegacyResearch>(communication_handler_, "dev", serial_port.serial_port, device_id.id)));
                qbrobotics_devices_found++;
            }
            if (qbrobotics_devices_found == 0) {
                std::cerr << "[scanForDevices] no qbrobotics devices found" << std::endl;
            }
        }
    }
    return qbrobotics_devices_found;

}


int qbSoftHandHandler::OpenSerialPort(const std::string &serial_port) {
    if (!std::regex_match(serial_port, std::regex("/dev/ttyUSB[[:digit:]]+"))) {
        return -1;
    }
    if(communication_handler_->openSerialPort(serial_port) < 0){
        std::cerr << "Not able to open: " << serial_port << " serial port";
        return -1;
    }
    std::cout << "Opened: " << serial_port << " serial port"<< std::endl;
    return 0;
}
