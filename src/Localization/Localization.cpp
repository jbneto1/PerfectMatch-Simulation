#include "Localization.h"

Localization::Localization(Logger &logger, AMRController &controller, const double control_cycle, const int maxIters)
        : logger(logger),
          controller(
                  controller),
          dt(control_cycle), PM(logger) {
    // Initialize the localization system
    firstIter = true;
    logger.debug("Localization system initialized.");
}

void Localization::processData(const std::array<int, 4> &encoders, const Pose &GT,
                               std::array<LaserPoint, 720> &lidarData) {
    static double runtime = 0;
    static double runtimePrevious = 0;

    groundTruth = GT;

    if (firstIter) {
        PM.setPose(GT);
        firstIter = false;
    }
    runtime += dt;
    double freq = 1 / (runtime - runtimePrevious);
    logger.fileLog("------------------------------------------");
    PM.setFreq(freq);
    Pose matchedPose = PM.match(lidarData); // Note the match result
    runtimePrevious = runtime;

    estimatedPose = matchedPose;

    logger.fileLog(fmt::format("gtX [m]: {:.2f}, gtY [m]: {:.2f}, gtTheta [deg]: {:.2f}",
                            groundTruth.getX(),
                            groundTruth.getY(),
                            radToDeg(groundTruth.getTheta()),
                            freq));

    logger.fileLog(fmt::format("pmX [m]: {:.2f}, pmY [m]: {:.2f}, pmTheta [deg]: {:.2f}",
                            matchedPose.getX(),
                            matchedPose.getY(),
                            radToDeg(matchedPose.getTheta()),
                            freq));

    logger.fileLog(fmt::format("ex [m]: {:.2f}, ey [m]: {:.2f}, etheta [deg]: {:.2f}, PM-Hz: {:.2f}",
                            (groundTruth.getX() - matchedPose.getX()),
                            (groundTruth.getY() - matchedPose.getY()),
                            radToDeg(diffAngle(groundTruth.getTheta(), matchedPose.getTheta())),
                            freq));

    logger.fileLog(fmt::format("Freq [Hz]: {:.2f}", freq));

    logger.trace("Data processed for Localization");
}

Pose Localization::getPose() {
    return (estimatedPose);
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

Pose Localization::getGTPose() {
    return groundTruth;
}


