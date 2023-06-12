//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_LOCALIZATION_H
#define PERFECTMATCH_SIMULATION_LOCALIZATION_H

#include <tuple>
#include <vector>
#include "Logger/logger.h"
#include "AMRController/AMRController.h"
#include "ExtendedKalmanFilter/ExtendedKalmanFilter.h"
#include "PerfectMatch/PerfectMatch.h"
#include "config.h"
#include "Eigen/Dense"

class Localization {
public:
    Localization(Logger &logger, AMRController &controller, const double control_cycle);

    void processData(const std::array<int, 4> &encoders, const Pose &GT, const std::array<double, 720> &lidarData);
    Pose getPose();
    void setPose(Pose &startPose);

    void setEncoders(const std::array<int, 4> &encoders);

private:
    Logger &logger;
    AMRController &controller;
    ExtendedKalmanFilter EKF;
    PerfectMatch PM;
    Pose groundTruth;
    Pose estimatedPose;
    const double dt;

    void updatePose(const std::array<int, 4> &encoders);
    void odometry(const std::array<int, 4> &encoders);
};


#endif //PERFECTMATCH_SIMULATION_LOCALIZATION_H
