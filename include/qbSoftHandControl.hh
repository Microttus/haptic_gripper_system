//
// Created by Martin Økter on 25/09/2023.
//

#ifndef UR5_QBHAND_CONTROL_QBSOFTHANDCONTROL_HH
#define UR5_QBHAND_CONTROL_QBSOFTHANDCONTROL_HH

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

class qbSoftHandControl{
public:
    qbSoftHandControl();
    ~qbSoftHandControl();

    std::vector<int16_t> GetControlReference();
    std::vector<int16_t> GetCurrents();
    std::vector<int16_t> GetPositions();
    std::vector<int16_t> GetVelocities();
    std::vector<int16_t> GetAccelerations();

    bool SetMotorStates(bool active);
    bool GetMotorStates();
    void SetGripValue(int[2] &control_ref);

    std::vector<float> GetPositionPID();
    std::vector<float> GetCurrentPID();

    int GetId();
    int GetStartupActivation();
    int GetInputMode();
    int GetControlMode();

    std::vector<uint8_t> GetEncoderResolution();
    std::vector<uint8_t> GetEncoderOffsets();
    std::vector<float> GetEncoderMultipliers();

    int GetUsePositionLimits();
    std::vector<int32_t> GetPositionLimits();
    std::vector<int32_t> GetPositionMaxSteps();
    int GetCurrentLimit();

private:
    qbSoftHandHandler SerialHandler_;

    std::map<int, std::shared_ptr<qbrobotics_research_api::qbSoftHandLegacyResearch>> soft_hands_;
    std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> device_ids_;           // IDs of connected devices
};

#endif //UR5_QBHAND_CONTROL_QBSOFTHANDCONTROL_HH
