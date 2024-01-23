//
// Created by jabra on 6/5/2023.
//

#ifndef AMR_PROJECT_CONFIG_H
#define AMR_PROJECT_CONFIG_H

constexpr int MAX_BUFFER_SIZE = 16256;
constexpr int SIMTWO_RECEIVE_PORT = 9000;
constexpr int SYNCMSG_RECEIVE_PORT = 9890;

constexpr int ENCODER_RESOLUTION = 3840;
constexpr double CONTROL_CYCLE = 0.025;
constexpr double STEP_SCALE = 0.04;
constexpr double LASER_RANGE = 360.0;
constexpr int LASER_RAYS = 720;
constexpr double A = 0.25 / 2 - 0.05;
constexpr double B = 0.155 / 2 + 0.015;
constexpr double C = A + B;
constexpr double R = 0.065 / 2;

#include <array>
#include <stdexcept>
#include <cmath>
#include "utils/utils.h"
#include <Eigen/Dense>

class Pose {
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

    operator Eigen::Vector3d() const {
        return Eigen::Vector3d(x, y, theta);
    }

    Pose operator+(const Eigen::Vector3d &vec) const {
        return Pose(x + vec[0], y + vec[1], normalizeAngle(theta + vec[2]));
    }

    Pose operator-(const Eigen::Vector3d &vec) const {
        return Pose(x - vec[0], y - vec[1], diffAngle(theta, vec[2]));
    }

    Pose &operator=(const Eigen::Vector3d &vec) {
        x = vec[0];
        y = vec[1];
        theta = vec[2];
        return *this;
    }

    Pose operator-(const Pose &other) const {
        double dx = x - other.getX();
        double dy = y - other.getY();
        double dtheta = diffAngle(theta, other.getTheta());
        // Normalize theta to be between -pi and pi.
        return Pose(dx, dy, dtheta);
    }

    Pose operator+(const Pose &other) const {
        double dx = x + other.getX();
        double dy = y + other.getY();
        double dtheta = theta + other.getTheta();
        // Normalize theta to be between -pi and pi.
        dtheta = normalizeAngle(dtheta);
        return Pose(dx, dy, dtheta);
    }

    Pose &operator=(const std::array<double, 3> &arr) {
        x = arr[0];
        y = arr[1];
        theta = arr[2];
        return *this;
    }


};

class LaserPoint {
private:
    double d;
    double angle;
    double x;
    double y;
    double std_dev;

    double dx, dy, dtheta;

public:
    LaserPoint() : d(0), angle(0), x(0), y(0), std_dev(1) {}

    LaserPoint(double d, double angle, double x, double y, double std_dev = 1) : d(d), angle(angle), x(x), y(y),
                                                                                 std_dev(std_dev) {}

    double getD() const { return d; }

    double getAngle() const { return angle; }

    double getX() const { return x; }

    double getY() const { return y; }

    double getStdDev() const { return std_dev; }

    void setD(double d) { this->d = d; }

    void setAngle(double angle) { this->angle = angle; }

    void setX(double x) { this->x = x; }

    void setY(double y) { this->y = y; }

    void setStdDev(double std_dev) { this->std_dev = std_dev; }

    void setDx(const double dx) { this->dx = dx; }

    void setDy(const double dy) { this->dy = dy; }

    void setDtheta(const double dtheta) { this->dtheta = dtheta; }

    double getDx() const { return dx; }

    double getDy() const { return dy; }

    double getDtheta() const { return dtheta; }
};

#endif //AMR_PROJECT_CONFIG_H
