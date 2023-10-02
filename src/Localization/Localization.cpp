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

void Localization::processData_wo_PM(const std::array<int, 4> &encoders, const Pose &GT) {

    auto start = std::chrono::high_resolution_clock::now();

    if (firstIter) {
        EKF.setPose(GT);
        firstIter = false;
    }

    Eigen::Vector4d encs = {static_cast<double>(encoders[0]),
                        static_cast<double>(encoders[1]),
                        static_cast<double>(encoders[2]),
                        static_cast<double>(encoders[3])};

    forward_kinematics(encs);
    EKF.setPose(odometry());
    EKF.predict(speedsStates);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    logger.info("ProcessData w/o LiDAR [us]: " + std::to_string(duration.count()));

}

void Localization::processData_w_PM(const std::array<int, 4> &encoders, const Pose &GT,
                               std::array<LaserPoint, 720> &lidarData) {

    processData_wo_PM(encoders, GT);

    auto start = std::chrono::high_resolution_clock::now();

    auto temp = PMMatchingWithLimit(PM, lidarData, 25, std::chrono::milliseconds(10));

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    logger.info("PM [us]: " + std::to_string(duration.count()));

    start = std::chrono::high_resolution_clock::now();

    EKF.update(temp);

    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    logger.info("EKF update [us]: " + std::to_string(duration.count()));

}

Pose Localization::PMMatchingWithLimit(PerfectMatch &PM, std::array<LaserPoint, 720> &lidarData, int max_iter,
                                       std::chrono::milliseconds max_duration) {
    auto timeout_time = std::chrono::high_resolution_clock::now() + max_duration;
    Pose result = Pose();

    PM.setPose(EKF.getPose());

    for (int iter = 0; iter < max_iter; iter++) {
        result = PM.match(lidarData);
        if (std::chrono::high_resolution_clock::now() >= timeout_time) {
            return result;
        }
    }
    return result;
}

void Localization::setPose(const Pose &startPose) {
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


