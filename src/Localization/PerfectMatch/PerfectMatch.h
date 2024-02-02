#ifndef PERFECTMATCH_H
#define PERFECTMATCH_H

#include <vector>
#include <cmath>
#include <stdexcept>
#include <chrono>

#include "Eigen/Dense"
#include "Map/Map.h"
#include "config/config.h"
#include "Logger/logger.h"
#include "utils/utils.h"
#include "data_structures/data_structures.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

class PerfectMatch
{
public:
    PerfectMatch(Logger &logger, const Pose startPose = Pose(), const double stepScale = STEP_SCALE);

    Pose match(std::array<LaserPoint, 720> &data);

    void setPose(const Pose pose) { this->RobotPose = pose; }

    Pose getPose() const { return RobotPose; };

    double getError() const { return this->pmError; }

    void ProcessLaserPoints(std::array<LaserPoint, 720> &LaserPoints);
    void ProcessLaserPoints(std::array<LaserPoint, 720> &LaserPoints, const Pose &previousPose, const Pose &currentPose);

    void ProcessBBOutliers(std::array<LaserPoint, 720> &LaserPoints, std::vector<BoundingBox> &outliers);

    void setStep(const double stepScale) { this->stepScale = stepScale; }

    double getStep() const { return this->stepScale; }

private:
    void RotateAndTranslate(double &rx, double &ry, double px, double py, double tx, double ty, double st, double ct);

    Pose interpolatePose(const Pose &previousPose, const Pose &currentPose, const double alpha);

    int XTopixel(double x);

    int YTopixel(double y);

    void IterLaser(std::array<LaserPoint, 720> &LaserPoints);

    Map map;
    double meterToPixel;
    Pose RobotPose;
    double stepScale;
    static constexpr double degreeStep = LASER_RANGE / LASER_RAYS;
    double pmError;

    Logger &logger;

    // Constant members that represent the spatial-relationship between LiDAR Scanner and Camera in the robot
    const Vector3d t_LC;
    const Matrix3d Rx, Ry, Rz;
    const Matrix4d TH_LC;
    const Matrix3d K;
};

#endif // PERFECTMATCH_H