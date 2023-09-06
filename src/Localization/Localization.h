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
#include <Eigen/Dense>
#include <utils/utils.h>

class Localization {
public:
    Localization(Logger &logger);

    void processData(const std::array<int, 4> &encoders, const Pose &GT, std::array<LaserPoint, 720> &lidarData, bool laserData);

    Pose getPose() { return EKF.getPose(); };

    PerfectMatch& getPM() { return PM; };

    float getFreq() const { return freq; };

    void setPose(Pose &startPose);

    bool firstIter;
private:

    //Robot methods

    void forward_kinematics(const Eigen::Vector4d encs);
    void wSpeeds_estimation(const Eigen::Vector4d encs);
    Pose odometry();

    Logger &logger;
    ExtendedKalmanFilter EKF;
    PerfectMatch PM;
    const double dt;
    float freq = 0.0f;

    //Robot attributes
    Eigen::Matrix<double, 3, 1> speedsStates;
    Eigen::Matrix<double, 4, 1> wSpeeds;
    const double a, b, r;
    const double c;
    const Eigen::Matrix<double, 3, 4> forwardK_model;
};

#endif //PERFECTMATCH_SIMULATION_LOCALIZATION_H
