//
// Created by jabra on 5/16/2023.
//

#include "ExtendedKalmanFilter.h"


ExtendedKalmanFilter::ExtendedKalmanFilter() {
    // Initialize the filter
}

Pose ExtendedKalmanFilter::getPose() {
    return this->pose;
}

void ExtendedKalmanFilter::setPose(const Pose startPose) {
    pose = startPose;
}

void ExtendedKalmanFilter::predict() {
    //
}

void ExtendedKalmanFilter::update(const Pose measurement) {
//placeholder
    pose = measurement;

}

//
//void Localization::odometry(const std::array<int, 4> &encoders) {
//    std::array<double, 3> estSpeedStates;
//    std::array<double, 3> propagatedPose;
//
////    estSpeedStates = controller.mecanum.getEstimatedSpeedStates(encoders);
//
//    double cosTheta, sinTheta;
//
//    propagatedPose[2] = getPose().getTheta() + estSpeedStates[2] * dt;
//
//    cosTheta = cos(propagatedPose[2]);
//    sinTheta = sin(propagatedPose[2]);
//
//    propagatedPose[0] = getPose().getX() +
//                        (cosTheta * estSpeedStates[0] - sinTheta * estSpeedStates[1]) * dt;
//    propagatedPose[1] = getPose().getY() +
//                        (sinTheta * estSpeedStates[0] + cosTheta * estSpeedStates[1]) * dt;
//
//    estimatedPose = propagatedPose;
//}
//