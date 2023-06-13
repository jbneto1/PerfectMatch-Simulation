//
// Created by jabra on 6/5/2023.
//

#ifndef AMR_PROJECT_CONFIG_H
#define AMR_PROJECT_CONFIG_H

#define MAX_BUFFER_SIZE 16256
#define SIMTWO_RECEIVE_PORT 9000

#define ENCODER_RESOLUTION 1920
#define CONTROL_CYCLE 0.025

#include <array>
#include <stdexcept>

class Pose {
private:
    double x;
    double y;
    double theta;
    double err;

public:
    Pose() : x(0), y(0), theta(0) {}
    Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}

    double getX() const { return x; }
    double getY() const { return y; }
    double getTheta() const { return theta; }
    double getErr() const { return err; }

    void setX(double x) { this->x = x; }
    void setY(double y) { this->y = y; }
    void setTheta(double theta) { this->theta = theta; }
    void setErr(double err) { this->err = err; }

    Pose& operator=(const std::array<double, 3>& arr) {
        if (arr.size() != 3) {
            throw std::invalid_argument("Unmatched array size.");
        }
        x = arr[0];
        y = arr[1];
        theta = arr[2];
        return *this;
    }
};

struct LaserPoint {
    double x;
    double y;
    double d;
    double std;
};

// Define array types
using TDistMap = std::array<std::array<int, 640>, 480>;
using TGradMap = std::array<std::array<double, 640>, 480>;
using TLaserPoints = std::array<LaserPoint, 720>;  // Assuming there are 720 laser points



#endif //AMR_PROJECT_CONFIG_H
