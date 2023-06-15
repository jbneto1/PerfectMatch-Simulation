// Localization.cpp

#include "Localization.h"

Localization::Localization(Logger &logger, AMRController &controller, const double control_cycle, const int maxIters)
        : logger(logger),
          controller(
                  controller),
          dt(control_cycle), PM(logger, "C:\\Users\\jabra\\Documents\\GitHub\\PerfectMatch-Simulation\\src\\Localization\\PerfectMatch\\Map\\RAFmap.png") {
    // Initialize the localization system
    logger.debug("Localization system initialized");
}

void Localization::processData(const std::array<int, 4> &encoders, const Pose &GT,
                               std::array<LaserPoint, 720> &lidarData) {
    logger.debug("Processing data for Localization. Encoders: (" + std::to_string(encoders[0]) + ", " +
                 std::to_string(encoders[1]) + ", " + std::to_string(encoders[2]) + ", " + std::to_string(encoders[3]) +
                 "), GT: (" + std::to_string(GT.getX()) + ", " + std::to_string(GT.getY()) + "), LidarData size: " +
                 std::to_string(lidarData.size()));
    static double runtime = 0;
    runtime += dt;
    groundTruth = GT;

    if (firstIter = true) {
        PM.setPose(GT);
        firstIter = false;
    }
    Pose matchedPose = PM.match(lidarData); // Note the match result

    // Log ground truth and matched pose data in csv format
    logger.fileLog(fmt::format("{},{},{},{},{},{},{}", groundTruth.getX(), groundTruth.getY(), groundTruth.getTheta(),
                            matchedPose.getX(), matchedPose.getY(), matchedPose.getTheta(), runtime));
    logger.trace("Data processed for Localization");
}

Pose Localization::getPose() {
    return (EKF.getPose());
}

void Localization::setPose(Pose &startPose) {
    logger.info("Setting pose for Localization to: (" + std::to_string(startPose.getX()) + ", " +
                std::to_string(startPose.getY()) + ", " + std::to_string(startPose.getTheta()) + ")");
    EKF.setPose(startPose);
    logger.debug("Pose set for Localization");
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
