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
    Localization(Logger &logger, AMRController &controller, const double control_cycle, const int maxIters);

    void processData(const std::array<int, 4> &encoders, const Pose &GT, std::array<LaserPoint, 720> &lidarData);

    Pose getPose();

    PerfectMatch& getPM() { return PM;}

    Pose getGTPose();

    void setPose(Pose &startPose);

    void setEncoders(const std::array<int, 4> &encoders);

    bool firstIter;

private:
    Logger &logger;
    AMRController &controller;
    ExtendedKalmanFilter EKF; //TODO organize and unify the groundtruth and estimated poses
    //TODO (differentiate EKF, PM estimate, encoder estimate, localization estimate)
    PerfectMatch PM;
    Pose groundTruth;
    Pose estimatedPose;
    const double dt;

    void updatePose(const std::array<int, 4> &encoders);

    void odometry(const std::array<int, 4> &encoders);


};


#endif //PERFECTMATCH_SIMULATION_LOCALIZATION_H
