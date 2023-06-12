// Localization.cpp

#include "Localization.h"

Localization::Localization(Logger &logger, AMRController &controller, const double control_cycle) : logger(logger),
                                                                                                    controller(
                                                                                                            controller),
                                                                                                    dt(control_cycle) {
    // Initialize the localization system
}

void Localization::processData(const std::array<int, 4> &encoders, const Pose &GT,
                               const std::array<double, 720> &lidarData) {
    static double runtime = 0;
    runtime += dt;
    groundTruth = GT;
    // odometry(encoders);
    // EKF.predict();
    Pose matchedPose = PM.match(lidarData); // Note the match result
    // EKF.update();

    // Log ground truth and matched pose data in csv format
    logger.info(fmt::format("{},{},{},{},{},{},{}", groundTruth.getX(), groundTruth.getY(), groundTruth.getTheta(),
                             matchedPose.getX(), matchedPose.getY(), matchedPose.getTheta(), runtime));
}

Pose Localization::getPose() {
    return (EKF.getPose());
}

void Localization::setPose(Pose &startPose) {
    EKF.setPose(startPose);
}


void Localization::odometry(const std::array<int, 4> &encoders) {
    std::array<double, 3> estSpeedStates;
    std::array<double, 3> propagatedPose;

//    estSpeedStates = controller.mecanum.getEstimatedSpeedStates(encoders);

    double cosTheta, sinTheta;

    propagatedPose[2] = getPose().getTheta() + estSpeedStates[2] * dt;

    cosTheta = cos(propagatedPose[2]);
    sinTheta = sin(propagatedPose[2]);

    propagatedPose[0] = getPose().getX() +
                        (cosTheta * estSpeedStates[0] - sinTheta * estSpeedStates[1]) * dt;
    propagatedPose[1] = getPose().getY() +
                        (sinTheta * estSpeedStates[0] + cosTheta * estSpeedStates[1]) * dt;

    estimatedPose = propagatedPose;
}
