//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_LOCALIZATION_H
#define PERFECTMATCH_SIMULATION_LOCALIZATION_H

#include <tuple>
#include <vector>
#include <Logger/logger.h>
#include <AMRController/AMRController.h>
#include "ExtendedKalmanFilter/ExtendedKalmanFilter.h"
#include "PerfectMatch/PerfectMatch.h"
#include <config/config.h>
#include <eigen-3.4.0/Eigen/Dense>
#include <utils/utils.h>

class Localization {
public:
    Localization(Logger &logger, const double control_cycle);

    void processData(const std::array<int, 4> &encoders, const Pose &GT, std::array<LaserPoint, 720> &lidarData);

    Pose getPose() { return EKF.getPose(); };

    PerfectMatch& getPM() { return PM; };

    Pose getGTPose() const { return groundTruth; };

    float getFreq() const { return freq; };

    void setPose(Pose &startPose);

    bool firstIter;
private:
    Logger &logger;
    ExtendedKalmanFilter EKF;
    PerfectMatch PM;
    Pose groundTruth;
    const double dt;
    float freq = 0.0f;
};

#endif //PERFECTMATCH_SIMULATION_LOCALIZATION_H
