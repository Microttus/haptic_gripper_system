//
// Created by Martin Økter on 07/10/2023.
//

#include "../include/qbSoftHandControl.hh"

#include "../lib/qbdevice-api-7.x.x/qbrobotics-driver/libs/research/include/qbrobotics_research_api/qbsofthand_research_api.h"

#include <iostream>
#include <string>
#include <vector>

qbSoftHandControl::qbSoftHandControl(std::map<int, std::shared_ptr<qbrobotics_research_api::qbSoftHandLegacyResearch> > *device_info, std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> *device_ids_in_) {

    qbSoftHandControl::soft_hands_ = *device_info;
    qbSoftHandControl::device_ids_ = *device_ids_in_;
}

qbSoftHandControl::~qbSoftHandControl() {

}

std::vector <int16_t> qbSoftHandControl::GetControlReference() {
    std::vector<int16_t> control_references;
    soft_hands_.at((int)id.id)->getControlReferences(control_references);
    for (auto &reference:control_references){
        std::cout << reference << " ";
    }
}

std::string qbSoftHandControl::GetDeviceInfo() {
    std::string info_string;
    soft_hands_.at((int)id.id)->getInfo(INFO_ALL, info_string);
    std::cout << info_string << std::endl << "----" << std::endl;
    return std::string();
}

std::vector<int16_t> qbSoftHandControl::GetCurrents() {

    std::vector<int16_t> currents;

    soft_hands_.at((int)id.id)->getCurrents(currents);
    for (auto &current:currents){
        std::cout << current << " ";
    }

    return currents;
}

std::vector<int16_t> qbSoftHandControl::GetPositions() {

    std::vector<int16_t> positions__;

    soft_hands_.at((int)id.id)->getPositions(positions__);
    for (auto &position:positions__){
        std::cout << position << " ";
    }
    return positions__;
}

std::vector<int16_t> qbSoftHandControl::GetVelocities() {

    std::vector<int16_t> velocities__;

    soft_hands_.at((int)id.id)->getVelocities(velocities__);
    for (auto &velocity:velocities__){
        std::cout << velocity << " ";
    }
    return velocities__;
}

std::vector<int16_t> qbSoftHandControl::GetAccelerations() {

    std::vector<int16_t> accelerations__;

    soft_hands_.at((int)id.id)->getAccelerations(accelerations__);
    for (auto &acceleration:accelerations__){
        std::cout << acceleration << " ";
    }
    return accelerations__;
}

bool qbSoftHandControl::SetMotorStates(bool active) {

    if (soft_hands_.at((int)id.id)->setMotorStates(active) == 0){
        std::cout << "Motors are active";
    } else {
        std::cout << "Something went wrong while activating motors";
        break;
    }
    return false;
}

bool qbSoftHandControl::GetMotorStates() {
    bool activate = false;
    soft_hands_.at((int)id.id)->getMotorStates(activate);
    if(activate){
        std::cout << "Motors are active";
    } else {
        std::cout << "Motors are not active";
        break;
    }
    return activate;
}

void qbSoftHandControl::SetGripValue(int grip, int spread) {  //TODO: Ask Muri about this formulation
    int control_references[] = {grip, spread};
    soft_hands_.at((int)id.id)->setControlReferences(control_references);
    for (auto &control_reference:control_references){
        std::cout << control_reference << " ";
    }

}

std::vector<float> qbSoftHandControl::GetPositionPID() {

    std::vector<float> PID__;

    soft_hands_.at((int)id.id)->getParamPositionPID(PID__);
    for (auto &param:PID__){
        std::cout << param << " ";
    }

    return PID__;
}

std::vector<float> qbSoftHandControl::GetCurrentPID() {

    std::vector<float> PID__;

    soft_hands_.at((int)id.id)->getParamCurrentPID(PID__);
    for (auto &param:PID__){
        std::cout << param << " ";
    }
    return PID__;
}

int qbSoftHandControl::GetId() {

    uint8_t device_id__;

    soft_hands_.at((int)id.id)->getParamId(device_id__);
    std::cout << (int)device_id__ << " ";

    return device_id__;
}

int qbSoftHandControl::GetStartupActivation() {

    uint8_t activation;

    soft_hands_.at((int)id.id)->getParamStartupActivation(activation);
    std::cout << (int)activation;

    return activation;
}

int qbSoftHandControl::GetInputMode() {

    uint8_t input_mode;

    soft_hands_.at((int)id.id)->getParamInputMode(input_mode);
    std::cout << (int)input_mode;

    return input_mode;
}

int qbSoftHandControl::GetControlMode() {

    uint8_t control_mode;

    soft_hands_.at((int)id.id)->getParamControlMode(control_mode);
    std::cout << (int)control_mode;

    return control_mode;
}

std::vector<uint8_t> qbSoftHandControl::GetEncoderResolution() {

    std::vector<uint8_t> encoder_resolutions__;

    soft_hands_.at((int)id.id)->getParamEncoderResolutions(encoder_resolutions__);
    for (auto &param:encoder_resolutions__){
        std::cout << (int)param << " ";
    }
    return encoder_resolutions__;
}

std::vector<uint8_t> qbSoftHandControl::GetEncoderOffsets() {

    std::vector<int16_t> encoder_offsets__;

    soft_hands_.at((int)id.id)->getParamEncoderOffsets(encoder_offsets__);
    for (auto &offset:encoder_offsets__){
        std::cout << offset << " ";
    }
    return encoder_offsets__;
}

std::vector<float> qbSoftHandControl::GetEncoderMultipliers() {

    std::vector<float> encoder_multipliers__;

    soft_hands_.at((int)id.id)->getParamEncoderMultipliers(encoder_multipliers__);
    for (auto &param:encoder_multipliers__){
        std::cout << param << " ";
    }
    return encoder_multipliers__;
}

int qbSoftHandControl::GetUsePositionLimits() {

    uint8_t use_position_limits__;

    soft_hands_.at((int)id.id)->getParamUsePositionLimits(use_position_limits__);
    std::cout << (int)use_position_limits__;

    return use_position_limits__;
}

std::vector<int32_t> qbSoftHandControl::GetPositionLimits() {

    std::vector<int32_t> position_limits__;

    soft_hands_.at((int)id.id)->getParamPositionLimits(position_limits__);
    for (auto &param:position_limits__){
        std::cout << (int)param << " ";
    }
    return position_limits__;
}

std::vector<int32_t> qbSoftHandControl::GetPositionMaxSteps() {

    std::vector<int32_t> position_max_steps__;

    soft_hands_.at((int)id.id)->getParamPositionMaxSteps(position_max_steps__);
    for (auto &param:position_max_steps__){
        std::cout << (int)param << " ";
    }
    return position_max_steps__;
}

int qbSoftHandControl::GetCurrentLimit() {

    int16_t current_limit__;

    soft_hands_.at((int)id.id)->getParamCurrentLimit(current_limit__);
    std::cout << (int)current_limit__;
    return current_limit__;
}
