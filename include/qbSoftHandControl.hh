//
// Created by Martin Økter on 25/09/2023.
//

#ifndef UR5_QBHAND_CONTROL_QBSOFTHANDCONTROL_HH
#define UR5_QBHAND_CONTROL_QBSOFTHANDCONTROL_HH


class qbSoftHandControl{
public:
    qbSoftHandControl(auto const *device_info);
    ~qbSoftHandControl();

    std::string GetDeviceInfo();
    std::vector<int16_t> GetControlReference();
    std::vector<int16_t> GetCurrents();
    std::vector<int16_t> GetPositions();
    std::vector<int16_t> GetVelocities();
    std::vector<int16_t> GetAccelerations();

    bool SetMotorStates(bool active);
    bool GetMotorStates();
    void SetGripValue(int grip, int spread);

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

    std::map<int, std::shared_ptr<qbrobotics_research_api::qbSoftHandLegacyResearch>> soft_hands_;
    std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> device_ids_;           // IDs of connected devices
};

#endif //UR5_QBHAND_CONTROL_QBSOFTHANDCONTROL_HH
