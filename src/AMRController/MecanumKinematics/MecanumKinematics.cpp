//
// Created by jabra on 5/16/2023.
//

#include "MecanumKinematics.h"

MecanumKinematics::MecanumKinematics(const double a, const double b, const double r, const double dt) : a(a), b(b), r(r), dt(dt),
                                                                                                        inverseKinematicModel(
                                                                                                            (Eigen::Matrix<double, 4, 3>()
                                                                                                                 << 1,
                                                                                                             -1, -c,
                                                                                                             1, 1, c,
                                                                                                             1, 1, -c,
                                                                                                             1, -1, c)
                                                                                                                .finished()),
                                                                                                        forwardKinematicModel(
                                                                                                            (Eigen::Matrix<double, 3, 4>()
                                                                                                                 << 1,
                                                                                                             1, 1, 1,
                                                                                                             -1, 1, 1, -1,
                                                                                                             -1 / c, 1 / c, -1 / c, 1 / c)
                                                                                                                .finished())
{
}

Eigen::Vector4d MecanumKinematics::estimateWheelSpeeds(const std::array<int, 4> &encoders)
{
    Eigen::Vector4d estimatedWheelSpeeds;

    if (encoders.size() != 4)
        std::runtime_error("Encoders size incorrect.");

    for (int i = 0; i < encoders.size(); i++)
        estimatedWheelSpeeds[i] = ((std::numbers::pi * 2) / (ENCODER_RESOLUTION * dt * 2) * encoders[i]);

    return estimatedWheelSpeeds;
}

void MecanumKinematics::calculateForwardKinematics(const Eigen::Vector4d &estimatedWheelSpeeds)
{
    estimatedSpeedStates = r / 4 * (forwardKinematicModel * estimatedWheelSpeeds);
}

void MecanumKinematics::calculateInverseKinematics(const Eigen::Vector3d &referenceSpeedStates)
{
    referenceWheelSpeeds = (inverseKinematicModel * referenceSpeedStates) / r;
}

std::array<double, 3> MecanumKinematics::getEstimatedSpeedStates(const std::array<int, 4> &encoders)
{
    Eigen::Vector4d estimatedWheelSpeeds = estimateWheelSpeeds(encoders);
    calculateForwardKinematics(estimatedWheelSpeeds);
    return {estimatedSpeedStates[0], estimatedSpeedStates[1], estimatedSpeedStates[2]};
}

std::array<double, 4> MecanumKinematics::getReferenceWheelSpeeds(const std::array<double, 3> &referenceSpeedStates)
{
    Eigen::Vector3d eigenSpeedStates = {referenceSpeedStates[0], referenceSpeedStates[1], referenceSpeedStates[2]};
    calculateInverseKinematics(eigenSpeedStates);
    return {referenceWheelSpeeds[0], referenceWheelSpeeds[1], referenceWheelSpeeds[2], referenceWheelSpeeds[3]};
}
