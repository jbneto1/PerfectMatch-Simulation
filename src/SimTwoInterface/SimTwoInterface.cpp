//
// Created by jabra on 5/16/2023.
//

#include "SimTwoInterface.h"

SimTwoInterface::SimTwoInterface() {
    // Initialize the simulator
}

std::tuple<std::vector<double>, std::vector<double>> SimTwoInterface::getSensorData() {
    // Retrieve and return lidar and encoder data from the simulator
}

void SimTwoInterface::setWheelSpeeds(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
    // Send wheel speeds to the simulator
}

