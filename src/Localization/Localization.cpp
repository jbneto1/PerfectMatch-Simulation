#include "Localization.h"

Localization::Localization(Logger &logger, const double control_cycle)
        : logger(logger),
          dt(control_cycle), PM(logger) {
    // Initialize the localization system
    firstIter = true;
    logger.debug("Localization system initialized.");
}

Localization::Localization(Logger &logger, const double control_cycle, const int maxIters)
        : logger(logger),
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
        EKF.setPose(groundTruth);
        firstIter = false;
    }

    runtime += dt;
    double fq = 1 / (runtime - runtimePrevious);
    freq = fq;
    logger.fileLog("------------------------------------------");
    PM.setPose(EKF.getPose());
    EKF.update(PM.match(lidarData)); // Note the match result
    runtimePrevious = runtime;

    logger.trace("Data processed for Localization");
}

void Localization::setPose(Pose &startPose) {
    EKF.setPose(startPose);
}


