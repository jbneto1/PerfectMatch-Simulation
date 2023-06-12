//
// Created by jabra on 5/16/2023.
//

#include "AMRController.h"


AMRController::AMRController(Logger &logger) : logger(logger) {
    // Initialize the controller
}

std::tuple<double, double, double, double> AMRController::computeWheelSpeeds(Pose estimatedPose) {
    // Compute and return wheel speeds based on position and orientation
    return {0,0,0,0};
}
