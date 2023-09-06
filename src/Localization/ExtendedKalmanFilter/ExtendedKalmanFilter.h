//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H
#define PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H

#include <vector>
#include "config/config.h"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

class ExtendedKalmanFilter {
public:
    //Constructor
    ExtendedKalmanFilter();

    //Methods
    void predict();
    void update(const Pose measurement);

    //Getters
    Pose getPose();

    //Setters
    void setPose(const Pose startPose);

private:

    //Methods
    void odometry();

private:

    //EKF members
    Pose mu;
    Matrix3d Pk;
    const Matrix3d Qk;
    Matrix3d Sk;
    Matrix3d Fk;

    //PM variables
    Matrix3d Rk_PM;
    Matrix3d Hk_PM;

    //Mahalanobis threshold
    double mahalanobis_threshold;
    double dist_mahalanobis;

};

#endif //PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H
