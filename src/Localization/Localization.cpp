#include "Localization.h"

Localization::Localization(Logger &logger)
        : logger(logger),
          dt(CONTROL_CYCLE), PM(logger), a(A), b(B), c(C), r(R),
          forwardK_model(
                  (Eigen::Matrix<double, 3, 4>() << 1, 1, 1, 1,
                          -1, 1, 1, -1,
                          -1 / c, 1 / c, -1 / c, 1 / c)
                          .finished()) {
    firstIter = true;
}

void Localization::processData(const std::array<int, 4> &encoders, const Pose &GT) {
    static double runtime = 0;
    static double runtimePrevious = 0;

    if (firstIter) {
        EKF.setPose(GT);
        firstIter = false;
    }

    runtime += dt;
    double fq = 1 / (runtime - runtimePrevious);
    freq = fq;

    Eigen::Vector4d encs = {encoders[0], encoders[1], encoders[2], encoders[3]};

    forward_kinematics(encs);
    EKF.setPose(odometry());
    EKF.predict(speedsStates);


    runtimePrevious = runtime;
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

    Eigen::Vector4d encs = {encoders[0], encoders[1], encoders[2], encoders[3]};

    forward_kinematics(encs);
    EKF.setPose(odometry());
    EKF.predict(speedsStates);

//    if (laserData) {
//        auto temp = PMMatchingWithLimit(PM, lidarData, 10, std::chrono::milliseconds(2));
//        EKF.update(temp);
//    }

    runtimePrevious = runtime;
}

Pose Localization::PMMatchingWithLimit(PerfectMatch &PM, std::array<LaserPoint, 720> &lidarData, int max_iter,
                                       std::chrono::milliseconds max_duration) {
    auto timeout_time = std::chrono::high_resolution_clock::now() + max_duration;
    Pose result = Pose();

    for (int iter = 0; iter < max_iter; iter++) {
        result = PM.match(lidarData);
        if (std::chrono::high_resolution_clock::now() >= timeout_time) {
            return result;
        }
    }
    return result;
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
    tmp = 2 * M_PI / (ENCODER_RESOLUTION * dt);
    wSpeeds = {tmp * encs[0], tmp * encs[1], tmp * encs[2], tmp * encs[3]};
}

Pose Localization::odometry() {
    Pose propagatedPose = Pose();

    double cosTheta, sinTheta;


    cosTheta = cos(EKF.getPose().getTheta());
    sinTheta = sin(EKF.getPose().getTheta());

    propagatedPose.setX(EKF.getPose().getX() + (cosTheta * speedsStates[0] - sinTheta * speedsStates[1]) * dt);
    propagatedPose.setY(EKF.getPose().getY() + (sinTheta * speedsStates[0] + cosTheta * speedsStates[1]) * dt);
    propagatedPose.setTheta(EKF.getPose().getTheta() + speedsStates[2] * dt);
    return propagatedPose;
}


