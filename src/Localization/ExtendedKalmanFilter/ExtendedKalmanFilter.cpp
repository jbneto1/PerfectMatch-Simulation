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

void ExtendedKalmanFilter::update() {
    //
}

