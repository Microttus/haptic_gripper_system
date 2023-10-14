//
// Created by Martin Økter on 14/10/2023.
//

#include <iostream>
#include <string>
#include <unistd.h>
#include <memory>
#include <regex>
#include <map>

#include <qbrobotics_research_api/qbsofthand_research_api.h>

class qbSoftHandControl{
public:
    qbSoftHandControl(int hand_nr_, std::map<int, std::shared_ptr<qbrobotics_research_api::qbSoftHandLegacyResearch>> *device_info)
    : this_soft_hand_(*device_info)
    , dev_id_(hand_nr_)
    {

    }

    ~qbSoftHandControl()
    {

    }

    std::string GetDeviceInfo(){
        std::string info_string;
        this_soft_hand_.at(qbSoftHandControl::dev_id_)->getInfo(INFO_ALL, info_string);
        std::cout << info_string << std::endl << "----" << std::endl;
        return std::string();
    }

    std::vector<int16_t> GetControlReference();
    std::vector<int16_t> GetCurrents();
    std::vector<int16_t> GetPositions(){
        std::vector<int16_t> positions__;

        this_soft_hand_.at(qbSoftHandControl::dev_id_)->getPositions(positions__);
        for (auto &position:positions__){
            std::cout << position << " ";
        }
        return positions__;
    };
    std::vector<int16_t> GetVelocities();
    std::vector<int16_t> GetAccelerations();

    bool SetMotorStates(bool active){
        if (this_soft_hand_.at(qbSoftHandControl::dev_id_)->setMotorStates(active) == 0){
            std::cout << "Motors set state success" << std::endl;
        } else {
            std::cout << "Something went wrong while setting state of motors" << std::endl;
        }
        return false;
    };
    bool GetMotorStates();
    void SetGripValue(int grip, int spread){
        std::vector<int16_t> control_references;
        control_references.push_back(grip);
        control_references.push_back(spread);
        this_soft_hand_.at(qbSoftHandControl::dev_id_)->setControlReferences(control_references);
        for (auto &control_reference:control_references){
            std::cout << control_reference << " ";
        }
        std::cout << std::endl;
    };

    std::vector<float> GetPositionPID();
    std::vector<float> GetCurrentPID();

    int GetId();
    int GetStartupActivation();
    int GetInputMode();
    int GetControlMode();

    std::vector<uint8_t> GetEncoderResolution();
    std::vector<int16_t> GetEncoderOffsets();
    std::vector<float> GetEncoderMultipliers();

    int GetUsePositionLimits();
    std::vector<int32_t> GetPositionLimits();
    std::vector<int32_t> GetPositionMaxSteps();
    int GetCurrentLimit();

private:

    std::map<int, std::shared_ptr<qbrobotics_research_api::qbSoftHandLegacyResearch> > this_soft_hand_;
    std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> device_ids_;           // IDs of connected devices

    int dev_id_;
};

class qbSoftHandHandler{
public:
    qbSoftHandHandler()
    : communication_handler_(std::make_shared<qbrobotics_research_api::CommunicationLegacy>())
    {

    };

    ~qbSoftHandHandler(){
        for (qbSoftHandControl dev : device_list_){
            dev.SetMotorStates(false);
        }
        sleep(2);

        for (auto &port:serial_ports_) {
            if (communication_handler_->closeSerialPort(port.serial_port) == 0) {
                std::cout << "serial port " << port.serial_port << " closed" << std::endl;
            }
        }
    };

    int ScanForDevices(const int &max_repeats){
        if (qbSoftHandHandler::communication_handler_->listSerialPorts(serial_ports_) < 0) {
            std::cerr << "[scanForDevices] no serial ports found" << std::endl;
            return -1;
        }

        int qbrobotics_devices_found = 0;
        for(auto &serial_port:qbSoftHandHandler::serial_ports_){ // scan and open all the serial port
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

            if (qbSoftHandHandler::communication_handler_->listConnectedDevices(serial_port.serial_port, qbSoftHandHandler::device_ids_) >= 0) { // retrieved at least a qbrobotics device
                for(auto &device_id:qbSoftHandHandler::device_ids_) {

                    if (device_id.id == 120 || device_id.id == 0) {
                        std::cout << "Not valid device retrieved!" << std::endl;
                        continue;  // ID 120 is reserved, ID 0 is for sure an error
                    }

                    qbSoftHandHandler::soft_hands_detected_.insert(std::make_pair(static_cast<int>(device_id.id), std::make_shared<qbrobotics_research_api::qbSoftHandLegacyResearch>(communication_handler_, "dev", serial_port.serial_port, device_id.id)));
                    qbrobotics_devices_found++;
                }
                if (qbrobotics_devices_found == 0) {
                    std::cerr << "[scanForDevices] no qbrobotics devices found" << std::endl;
                }
            }
        }

        return qbrobotics_devices_found;
    };
    int OpenSerialPort(const std::string &serial_port){
        if (!std::regex_match(serial_port, std::regex("/dev/ttyUSB[[:digit:]]+"))) {
            return -1;
        }
        if(qbSoftHandHandler::communication_handler_->openSerialPort(serial_port) < 0){
            std::cerr << "Not able to open: " << serial_port << " serial port";
            return -1;
        }
        std::cout << "Opened: " << serial_port << " serial port"<< std::endl;
        return 0;
    };

    void testLib(){
        std::cout << "Test conducted with success" << std::endl;
    };

    std::vector<qbSoftHandControl> ReturnDeviceMap(){
        ScanForDevices(3);

        std::map<int, std::shared_ptr<qbrobotics_research_api::qbSoftHandLegacyResearch> >::iterator it_device_ = soft_hands_detected_.begin();

        while (it_device_ != soft_hands_detected_.end())
        {
            //std::cout << "Key: " << it_device_->first << ", Value: " << it_device_->second << std::endl;
            device_list_.push_back(qbSoftHandControl(it_device_->first,&soft_hands_detected_));

            ++it_device_;
        }

        return device_list_;
    };

private:
    std::shared_ptr<qbrobotics_research_api::Communication> communication_handler_;                 // Handler to manage the communication with qbdevices
    std::vector<serial::PortInfo> serial_ports_;
    std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> device_ids_;           // IDs of connected devices
    std::map<int, std::shared_ptr<qbrobotics_research_api::qbSoftHandLegacyResearch> > soft_hands_detected_;
    std::vector<qbSoftHandControl> device_list_;

};


// Main program

qbSoftHandHandler my_hands_;

int main() {

    std::vector<qbSoftHandControl> qbSoftHand_devices = my_hands_.ReturnDeviceMap();
    std::cout << "There are " << qbSoftHand_devices.size() << " available qbSoftHand Research available for control" << std::endl;

    qbSoftHand_devices[0].SetMotorStates(true);
    sleep(2);
    qbSoftHand_devices[0].SetGripValue(10000,0);
    sleep(2);
    qbSoftHand_devices[0].SetGripValue(0,0);
    sleep(2);

    std::cout << "Main functoin completed" << std::endl;
};