#ifndef PERFECTMATCH_H
#define PERFECTMATCH_H

#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include <chrono>

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Map/Map.h"
#include "config/config.h"
#include "Logger/logger.h"
#include "utils/utils.h"
#include "data_structures/data_structures.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

class PerfectMatch
{
public:
    PerfectMatch(Logger &logger, const Pose startPose = Pose(), const double stepScale = STEP_SCALE);

    Pose match(std::vector<LaserPoint> &data);

    void setPose(const Pose pose) { this->RobotPose = pose; }

    Pose getPose() const { return RobotPose; };

    double getError() const { return this->pmError; }

    void ProcessLaserPoints(std::vector<LaserPoint> &LaserPoints);
    void ProcessLaserPoints(std::vector<LaserPoint> &LaserPoints, const Pose &previousPose, const Pose &currentPose);

    void ProcessBBOutliers(std::vector<LaserPoint> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter);

    void setStep(const double stepScale) { this->stepScale = stepScale; }

    double getStep() const { return this->stepScale; }

    void setSafetyThreshold(const int factor) { this->safety_threshold = factor; }

    int getSafetyThreshold() const { return this->safety_threshold; }

private:
    void RotateAndTranslate(double &rx, double &ry, double px, double py, double tx, double ty, double st, double ct);

    Pose interpolatePose(const Pose &previousPose, const Pose &currentPose, const double alpha);

    int XTopixel(double x);

    int YTopixel(double y);

    void IterLaser(std::vector<LaserPoint> &LaserPoints);

    bool isPointInsideBB(const Vector2d &point, const BoundingBox &box);

    // void CorrectDistortion(Vector3d &point);

    Map map;
    double meterToPixel;
    Pose RobotPose;
    double stepScale;
    static constexpr double degreeStep = LASER_RANGE / LASER_RAYS;
    double pmError;

    Logger &logger;

    // Constant members that represent the spatial-relationship between LiDAR Scanner and Camera in the robot
    const Vector3d t_LC;
    const double yaw = M_PI_2;
    const double pitch = -M_PI_2;
    const double roll = 0;
    const Eigen::AngleAxisd Rx, Ry, Rz;
    const Matrix4d TH_LC;
    /*K = [fx, skew, px
            0, fy, py,
            0, 0, 1]*/
    const Matrix3d K;
    const VectorXd distCoeffs;
    int safety_threshold;
};

#endif // PERFECTMATCH_H