//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H
#define PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H

#include <vector>
#include <cmath>

#include "Eigen/Dense"
#include "Eigen/Core"

#include "config/config.h"
#include "data_structures/data_structures.h"

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

class ExtendedKalmanFilter {
public:
    //Constructor
    ExtendedKalmanFilter();

    //Methods
    void predict(const Eigen::Vector3d twist);
    void update(const Pose Zk);

    //Getters
    Pose getPose();
    double getQk() { return Qk.diagonal()[0]; };

    //Setters
    void setPose(const Pose startPose);
    void setQ(const double principalDiagonal);


private:

    //Methods
    void odometry();

private:

    //EKF members
    Pose mu;
    Matrix3d Pk;
    Matrix3d Qk;
    Matrix3d Sk;
    Matrix3d Fk;
    double dt;

    //PM variables
    Matrix3d Rk_PM;
    Matrix3d Hk_PM;

    //Mahalanobis threshold
    double mahalanobis_threshold;
    double dist_mahalanobis;

};

#endif //PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H
