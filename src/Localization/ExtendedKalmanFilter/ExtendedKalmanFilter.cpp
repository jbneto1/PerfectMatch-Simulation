//
// Created by jabra on 5/16/2023.
//

#include "ExtendedKalmanFilter.h"


ExtendedKalmanFilter::ExtendedKalmanFilter() : Qk((Matrix3d() << 0.1, 0, 0,
        0, 0.1, 0,
        0, 0, 0.1).finished()) {

    mu = Pose();

    // Initialize the filter
    Fk = Matrix3d::Identity();
    Pk = Matrix3d::Identity();
    Pk.diagonal() << 1000, 1000, 1000;
    Sk = Matrix3d().Zero(3, 3);

    //Initialization aruco
    Hk_PM = Matrix3d::Identity(3, 3);
    Rk_PM = Matrix3d::Identity(3, 3);
    Rk_PM.diagonal() << 3.2, 3.2, 3.2;
}

Pose ExtendedKalmanFilter::getPose() {
    return this->mu;
}

void ExtendedKalmanFilter::setPose(const Pose startPose) {
    mu = startPose;
}

void ExtendedKalmanFilter::predict(const Eigen::Vector3d twist) {
    //odometry variable is already ^Xk_k_1
    double deriv_fx;
    double deriv_fy;
    double a = cos(mu.getTheta());
    double b = sin(mu.getTheta());

    deriv_fx = (-b * twist[0] - a * twist[1]);
    deriv_fy = (-b * twist[0] + a * twist[1]);

    Fk(0, 2) = deriv_fx;
    Fk(1, 2) = deriv_fy;

    Pk = Fk * Pk * Fk.transpose() + Qk;
}

void ExtendedKalmanFilter::update(const Pose Zk) {
    Vector3d innovation_PM;

    innovation_PM(0, 0) = Zk.getX() - mu.getX();
    innovation_PM(1, 0) = Zk.getY() - mu.getY();
    innovation_PM(2, 0) = normalizeAngle(Zk.getTheta() - mu.getTheta());

    Sk = Hk_PM * Pk * Hk_PM.transpose() + Rk_PM;
    Matrix3d Kg = Pk * Hk_PM.transpose() * Sk.inverse();
    Vector3d temp = (Kg * innovation_PM);

    mu.setX(mu.getX() + temp(0));
    mu.setY(mu.getY() + temp(1));
    mu.setTheta(normalizeAngle(mu.getTheta() + normalizeAngle(temp(2))));
    Pk = (Matrix3d::Identity() - Kg * Hk_PM) * Pk;
}