#include "Localization.h"

Localization::Localization(Logger &logger)
        : logger(logger),
          dt(CONTROL_CYCLE), PM(logger), a(A), b(B), c(a + b), r(R),
          forwardK_model(
                  (Eigen::Matrix<double, 3, 4>() << 1, 1, 1, 1,
                          -1, 1, 1, -1,
                          -1 / c, 1 / c, -1 / c, 1 / c)
                          .finished()) {
    firstIter = true;
}


void Localization::processData(const std::array<int, 4> &encoders, const Pose &GT,
                               std::array<LaserPoint, 720> &lidarData) {
    static double runtime = 0;
    static double runtimePrevious = 0;

    if (firstIter) {
        EKF.setPose(GT);
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

void Localization::forward_kinematics(const Eigen::Vector4d encs) {

    wSpeeds_estimation(encs);

    speedsStates = r / 4 * (forwardK_model * wSpeeds);

}

void Localization::wSpeeds_estimation(const Eigen::Vector4d encs) {
    double tmp;
    tmp = 2 * M_PI / (ENCODER_RESOLUTION * dt * 2);
    wSpeeds = {tmp * encs[0], tmp * encs[1], tmp * encs[2], tmp * encs[3]};
}

Pose Localization::odometry() {
    Pose propagatedPose = Pose();

    static double cosTheta, sinTheta;

    propagatedPose.setTheta(EKF.getPose().getTheta() + speedsStates[2] * dt);

    cosTheta = cos(propagatedPose.getTheta());
    sinTheta = sin(propagatedPose.getTheta());

    propagatedPose.setX(EKF.getPose().getX() + (cosTheta * speedsStates[0] - sinTheta * speedsStates[1]) * dt);
    propagatedPose.setY(EKF.getPose().getY() + (sinTheta * speedsStates[0] + cosTheta * speedsStates[1]) * dt);

    return propagatedPose;
}


