#ifndef PERFECTMATCH_SIMULATION_LOCALIZATION_H
#define PERFECTMATCH_SIMULATION_LOCALIZATION_H

#include <array>
#include <chrono>
#include <Eigen/Dense>
#include <Logger/logger.h>
#include "PerfectMatch/PerfectMatch.h"
#include "ExtendedKalmanFilter/ExtendedKalmanFilter.h"

class Localization {
public:
    Localization(Logger &logger);

    void processData_w_PM(const std::array<int, 4> &encoders, const Pose &GT, std::array<LaserPoint, 720> &lidarData);
    void processData_wo_PM(const std::array<int, 4> &encoders, const Pose &GT);
    void setPose(const Pose &startPose);

    Pose getPose() { return EKF.getPose(); };
    PerfectMatch &getPM() { return PM; };
    ExtendedKalmanFilter &getEKF() { return EKF; };
    const double getFreq() const { return freq; };

    void setFreq(double frequency) { this->freq = frequency; }

private:
    // private methods
    Pose PMMatchingWithLimit(PerfectMatch &PM, std::array<LaserPoint, 720> &lidarData, int max_iter,
                             std::chrono::milliseconds max_duration);
    void forward_kinematics(const Eigen::Vector4d encs);
    void wSpeeds_estimation(const Eigen::Vector4d encs);
    void operationFrequency(double &runtime, double &runtimePrevious);
    Pose odometry();

    // private members
    Logger &logger;
    ExtendedKalmanFilter EKF;
    PerfectMatch PM;
    const double dt;
    double freq = 0.0;
    bool firstIter;

    Eigen::Matrix<double, 3, 1> speedsStates;
    Eigen::Matrix<double, 4, 1> wSpeeds;
    const double a, b, r; // Distance from the robot's center to the middle of the wheel projected in x and y axes, respectively.
    // r is radius of the wheel
    const double c; // a + b
    const Eigen::Matrix<double, 3, 4> forwardK_model;
};

#endif //PERFECTMATCH_SIMULATION_LOCALIZATION_H