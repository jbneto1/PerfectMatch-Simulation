//
// Created by jabra on 5/16/2023.
//
#include "SimTwoInterface/SimTwoInterface.h"
#include "Localization/Localization.h"
#include "AMRController/AMRController.h"

int main() {
    SimTwoInterface simulator;
    Localization localization;
    AMRController controller;

    while (true) {
        auto [lidarData, encoderData] = simulator.getSensorData();
        auto [x, y, theta] = localization.processData(lidarData, encoderData);
        auto [frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed] = controller.computeWheelSpeeds(x, y, theta);
        simulator.setWheelSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    return 0;
}
