#ifndef AMR_PROJECT_DATA_STRUCTURE_H
#define AMR_PROJECT_DATA_STRUCTURE_H

#include <array>
#include <stdexcept>
#include <cmath>

#include "Eigen/Dense"

#include "utils/utils.h"
#include "config/config.h"

using Eigen::Vector2d;

class Pose
{
private:
    double x;
    double y;
    double theta;

public:
    Pose() : x(0), y(0), theta(0) {}

    Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}

    Pose(const Eigen::Vector3d &vec) : x(vec[0]), y(vec[1]), theta(vec[2]) {}

    double getX() const { return x; }

    double getY() const { return y; }

    double getTheta() const { return theta; }

    double getThetaDeg() const { return (theta * 180 / M_PI); }

    void setX(double x) { this->x = x; }

    void setY(double y) { this->y = y; }

    void setTheta(double theta) { this->theta = normalizeAngle(theta); }

    operator Eigen::Vector3d() const
    {
        return Eigen::Vector3d(x, y, theta);
    }

    Pose operator+(const Eigen::Vector3d &vec) const
    {
        return Pose(x + vec[0], y + vec[1], normalizeAngle(theta + vec[2]));
    }

    Pose operator-(const Eigen::Vector3d &vec) const
    {
        return Pose(x - vec[0], y - vec[1], diffAngle(theta, vec[2]));
    }

    Pose &operator=(const Eigen::Vector3d &vec)
    {
        x = vec[0];
        y = vec[1];
        theta = vec[2];
        return *this;
    }

    Pose operator-(const Pose &other) const
    {
        double dx = x - other.getX();
        double dy = y - other.getY();
        double dtheta = diffAngle(theta, other.getTheta());
        // Normalize theta to be between -pi and pi.
        return Pose(dx, dy, dtheta);
    }

    Pose operator+(const Pose &other) const
    {
        double dx = x + other.getX();
        double dy = y + other.getY();
        double dtheta = theta + other.getTheta();
        // Normalize theta to be between -pi and pi.
        dtheta = normalizeAngle(dtheta);
        return Pose(dx, dy, dtheta);
    }

    Pose &operator=(const std::array<double, 3> &arr)
    {
        x = arr[0];
        y = arr[1];
        theta = arr[2];
        return *this;
    }
};

class LaserPoint
{
private:
    double d;
    double angle;
    double x;
    double y;
    double std_dev;
    bool valid;
    bool draw;
    u_int index;

    double dx, dy, dtheta;

    // Image coordinates
    Vector2d pImgPx;

public:
    LaserPoint() : d(0), angle(0), x(0), y(0), std_dev(1), valid(true), draw(true), index(-1), dx(0), dy(0), dtheta(0), pImgPx(Vector2d::Zero()) {}

    LaserPoint(double d, double angle, double x, double y, u_int idx, double std_dev = 1, bool beamValid = true) : d(d), angle(angle), x(x), y(y),
                                                                                                                   std_dev(std_dev), valid(beamValid), draw(true), index(idx), dx(0), dy(0), dtheta(0), pImgPx(Vector2d::Zero()) {}

    double getD() const { return d; }

    double getAngle() const { return angle; }

    double getX() const { return x; }

    double getY() const { return y; }

    double getStdDev() const { return std_dev; }

    bool getIsBeamValid() const { return valid; };

    u_int getBeamIndex() const { return index; };

    void setBeamIndex(u_int idx) { this->index = idx; };

    void setIsBeamValid(bool valid) { this->valid = valid; };

    void setD(double d) { this->d = d; }

    void setAngle(double angle) { this->angle = angle; }

    void setX(double x) { this->x = x; }

    void setY(double y) { this->y = y; }

    void setStdDev(double std_dev) { this->std_dev = std_dev; }

    void setDx(double dx) { this->dx = dx; }

    void setDy(double dy) { this->dy = dy; }

    void setDtheta(double dtheta) { this->dtheta = dtheta; }

    double getDx() const { return dx; }

    double getDy() const { return dy; }

    double getDtheta() const { return dtheta; }

    Vector2d getImgPts() const { return pImgPx; }

    void setImgPts(Vector2d imgPtsPx) { this->pImgPx = imgPtsPx; }

    void setDraw(bool draw) { this->draw = draw; }

    bool getDraw() const { return draw; }
};

struct BoundingBox
{
    int class_id;
    double conf, x, y, width, height;
};

struct VisualizationData
{
    Pose groundTruth = {};
    Pose estimatedPose = {};
    std::vector<LaserPoint> laserPoint = {};
    bool drawLaser = false;
    int laserRejectCounter = 0;

    float freq_localization = 0;

    Pose estimatedPoseOutliers = {};
    bool drawLaserOutliers = false;
    std::vector<LaserPoint> laserPointOutliers = {};
    std::vector<BoundingBox> bboxes = {};
};

struct LocalizationUpdateData
{

    bool hasStepChanged = false;
    bool hasQkChanged = false;
    bool hasPoseChanged = false;
    bool hasSafetyChanged = false;

    double stepSet = 0.0;
    double stepGet = 0.0;

    double Qk_covarianceSet = 0.0;
    double Qk_covarianceGet = 0.0;

    int safetyThresholdSet = SAFETY_THRESHOLD;
    int safetyThresholdGet = SAFETY_THRESHOLD;

    Pose newPose = {};

    double PMError = 0.0;
    double PMError_semantics = 0.0;
};

#endif