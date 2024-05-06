#ifndef PERFECTMATCH_SIMULATION_LOCALIZATION_H
#define PERFECTMATCH_SIMULATION_LOCALIZATION_H

#include <array>
#include <chrono>

#include "Eigen/Dense"

#include "Logger/logger.h"
#include "PerfectMatch/PerfectMatch.h"
#include "ExtendedKalmanFilter/ExtendedKalmanFilter.h"

class Localization
{
public:
    Localization(Logger &logger);

    void processData_w_PM(const std::array<int, 4> &encoders, const Pose &GT, std::vector<LaserPoint> &lidarData);
    void processData_wo_PM(const std::array<int, 4> &encoders, const Pose &GT);

    void processData_w_PM(const std::array<int, 4> &encoders, const Pose &GT, std::vector<LaserPoint> &lidarData, const double dt);
    void processData_wo_PM(const std::array<int, 4> &encoders, const Pose &GT, const double dt);

    Pose extrinsic_calibrate_GT(Pose &uncalibrated_pose);
    Pose extrinsic_calibrate_PM(Pose &uncalibrated_pose);

    Pose getPose() { return EKF.getPose(); };
    Pose getPreviousPose() { return previousPose; };
    PerfectMatch &getPM() { return PM; };
    ExtendedKalmanFilter &getEKF() { return EKF; };
    const double getFreq() const { return freq; };
    const bool getFirstFlag() { return firstIter; };
    const double getCurrentDT() { return dt; };

    void setPose(const Pose &startPose);
    void setFreq(double frequency) { this->freq = frequency; }
    void setPreviousPose(const Pose currentPose) { this->previousPose = currentPose; };
    void setNewDT(const double newDT)
    {
        this->dt = newDT;
        EKF.setEKFdt(newDT);
    };

private:
    // private methods
    Pose PMMatchingWithLimit(PerfectMatch &PM, std::vector<LaserPoint> &lidarData, int max_iter,
                             std::chrono::milliseconds max_duration);
    void forward_kinematics(const Eigen::Vector4d encs);
    void wSpeeds_estimation(const Eigen::Vector4d encs);
    void operationFrequency(double &runtime, double &runtimePrevious);
    Pose odometry();

    // private members
    double dt;
    Logger &logger;
    ExtendedKalmanFilter EKF;
    PerfectMatch PM;
    double freq = 0.0;
    bool firstIter;

    Pose previousPose;

    Eigen::Matrix<double, 3, 1> speedsStates;
    Eigen::Matrix<double, 4, 1> wSpeeds;
    const double a, b, r; // Distance from the robot's center to the middle of the wheel projected in x and y axes, respectively.
    // r is radius of the wheel
    const double c; // a + b
    const Eigen::Matrix<double, 3, 4> forwardK_model;
    Eigen::Matrix2d offsetRot;
    Eigen::Vector2d offsetTrans;
};

#endif // PERFECTMATCH_SIMULATION_LOCALIZATION_H