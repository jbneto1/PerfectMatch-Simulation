#ifndef PERFECTMATCH_H
#define PERFECTMATCH_H

#include <vector>
#include <cmath>
#include <stdexcept>
#include "Map/Map.h"
#include "config.h"
#include "Logger/logger.h"

class PerfectMatch {
public:
    PerfectMatch(Logger &logger, const Pose startPose = Pose(), const int maxIters = 10,
                 const int cErr = 100, const double stepScale = 0.01);


    Pose match(std::array<LaserPoint, 720> &data);

    void setPose(const Pose pose) { this->RobotPose = pose; }

private:
    void RotateAndTranslate(double &rx, double &ry, double px, double py, double tx, double ty, double st, double ct);

    void MEstGradMap();

    void ProcessLaserPoints(std::array<LaserPoint, 720>& LaserPoints);

    int XTopixel(double x);

    int YTopixel(double y);

    double d_err(double d);

    void IterLaser(const std::array<LaserPoint, 720> &LaserPoints);

    Map map;
    double PixelSize;
    double PixelScale;
    Pose RobotPose;
    const int c_err; // Need to know what is this about
    const double stepScale;
    const int maxIters;
    static constexpr double degreeStep = LASER_RANGE / LASER_RAYS;
    Logger &logger;
};

#endif // PERFECTMATCH_H