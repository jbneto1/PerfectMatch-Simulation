//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_MECANUMKINEMATICS_H
#define PERFECTMATCH_SIMULATION_MECANUMKINEMATICS_H

#include "third_party/eigen-3.4.0/Eigen/Dense"
#include "config/config.h"

class MecanumKinematics {
public:
    MecanumKinematics(const double a, const double b, const double r, const double dt);

    std::array<double, 3> getEstimatedSpeedStates(const std::array<int, 4> &encoders);
    std::array<double, 4> getReferenceWheelSpeeds(const std::array<double, 3> &referenceSpeedStates);

private:
    void calculateForwardKinematics(const Eigen::Vector4d &estimatedWheelSpeeds);
    void calculateInverseKinematics(const Eigen::Vector3d &referenceSpeedStates);

    Eigen::Vector4d estimateWheelSpeeds(const std::array<int, 4> &encoders);

    const double dt; //control cycle

    const Eigen::Matrix<double, 4, 3> inverseKinematicModel;
    const Eigen::Matrix<double, 3, 4> forwardKinematicModel;

    const double a, b, r;
    const double c = a + b;

    Eigen::Vector3d estimatedSpeedStates;
    Eigen::Vector4d referenceWheelSpeeds;
};


#endif //PERFECTMATCH_SIMULATION_MECANUMKINEMATICS_H
