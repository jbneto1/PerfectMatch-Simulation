//
// Created by jabra on 6/5/2023.
//

#ifndef AMR_PROJECT_CONFIG_H
#define AMR_PROJECT_CONFIG_H

constexpr int MAX_BUFFER_SIZE = 16256;
constexpr int SIMTWO_RECEIVE_PORT = 9000;

constexpr int ENCODER_RESOLUTION = 1920;
constexpr double CONTROL_CYCLE = 0.025;
constexpr int MAX_ITERS = 10;
constexpr double LASER_RANGE = 360.0;
constexpr int LASER_RAYS = 720;

#include <array>
#include <stdexcept>
#include <cmath>

class Pose {
private:
    double x;
    double y;
    double theta;
    double err;

public:
    Pose() : x(0), y(0), theta(0), err(0) {}

    Pose(double x, double y, double theta) : x(x), y(y), theta(theta), err(0) {}

    double getX() const { return x; }

    double getY() const { return y; }

    double getTheta() const { return theta; }

    double getThetaDeg() const { return (theta * 180 / M_PI); }

    double getErr() const { return err; }

    void setX(double x) { this->x = x; }

    void setY(double y) { this->y = y; }

    void setTheta(double theta) { this->theta = theta; }

    void setErr(double err) { this->err = err; }

    Pose operator-(const Pose &other) const {
        double dx = x - other.getX();
        double dy = y - other.getY();
        double dtheta = theta - other.getTheta();
        // Normalize theta to be between -pi and pi.
        dtheta = fmod(dtheta, 2 * M_PI);
        if (dtheta >= M_PI) {
            dtheta -= 2 * M_PI;
        } else if (dtheta < -M_PI) {
            dtheta += 2 * M_PI;
        }
        return Pose(dx, dy, dtheta);
    }

    Pose &operator=(const std::array<double, 3> &arr) {
        if (arr.size() != 3) {
            throw std::invalid_argument("Unmatched array size.");
        }
        x = arr[0];
        y = arr[1];
        theta = arr[2];
        return *this;
    }
};

class Encoders {
private:
    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;

public:
    Encoders() : frontLeft(0), frontRight(0), backLeft(0), backRight(0) {}

    Encoders(double frontLeft, double frontRight, double backLeft, double backRight) :
            frontLeft(frontLeft), frontRight(frontRight), backLeft(backLeft), backRight(backRight) {}

    double getFrontLeft() const { return frontLeft; }

    double getFrontRight() const { return frontRight; }

    double getBackLeft() const { return backLeft; }

    double getBackRight() const { return backRight; }

    void setFrontLeft(double frontLeft) { this->frontLeft = frontLeft; }

    void setFrontRight(double frontRight) { this->frontRight = frontRight; }

    void setBackLeft(double backLeft) { this->backLeft = backLeft; }

    void setBackRight(double backRight) { this->backRight = backRight; }
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
