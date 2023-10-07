//
// Created by Martin Økter on 07/10/2023.
//

#ifndef UR5_QBHAND_CONTROL_QBSOFTHANDHANDLER_HH
#define UR5_QBHAND_CONTROL_QBSOFTHANDHANDLER_HH

class qbSoftHandHandler{
public:
    qbSoftHandHandler();
    ~qbSoftHandHandler();

    void ScanForDevices(const int &max_repeats);
    void OpenSerialPort(const std::string &serial_port);

    std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> ReturnDeviceID();

private:
    std::shared_ptr<qbrobotics_research_api::Communication> communication_handler_;                 // Handler to manage the communication with qbdevices
    std::vector<serial::PortInfo> serial_ports_;
    std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> device_ids_;           // IDs of connected devices
};

#endif //UR5_QBHAND_CONTROL_QBSOFTHANDHANDLER_HH
