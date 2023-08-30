#ifndef PERFECTMATCH_H
#define PERFECTMATCH_H

#include <vector>
#include <cmath>
#include <stdexcept>
#include "Map/Map.h"
#include <config/config.h>
#include <Logger/logger.h>
#include <utils/utils.h>

class PerfectMatch {
public:
    PerfectMatch(Logger &logger, const Pose startPose = Pose(), const int maxIters = 100, const double stepScale = 0.005);

    Pose match(std::array<LaserPoint, 720> &data);

    void setPose(const Pose pose) { this->RobotPose = pose; }

    double getError() const { return this->RobotPose.getErr(); }

    float getFreq() const { return this->freq; }

    void setFreq(const float hz) { this->freq = hz; }

private:

    float freq = 0.0f;

    void RotateAndTranslate(double &rx, double &ry, double px, double py, double tx, double ty, double st, double ct);

    void ProcessLaserPoints(std::array<LaserPoint, 720> &LaserPoints);

    int XTopixel(double x);

    int YTopixel(double y);

    void IterLaser(std::array<LaserPoint, 720> &LaserPoints);

    Map map;
    double meterToPixel;
    double pixelToMeter;
    Pose RobotPose;
    const double stepScale;
    const int maxIters;
    static constexpr double degreeStep = LASER_RANGE / LASER_RAYS;
    Logger &logger;
};

#endif // PERFECTMATCH_H