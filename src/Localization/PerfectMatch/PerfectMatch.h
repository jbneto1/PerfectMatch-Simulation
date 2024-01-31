#ifndef PERFECTMATCH_H
#define PERFECTMATCH_H

#include <vector>
#include <cmath>
#include <stdexcept>
#include "Map/Map.h"
#include "config/config.h"
#include "Logger/logger.h"
#include "utils/utils.h"
#include <chrono>

class PerfectMatch {
public:
    PerfectMatch(Logger &logger, const Pose startPose = Pose(), const double stepScale = STEP_SCALE);

    Pose match(std::array<LaserPoint, 720> &data);

    void setPose(const Pose pose) { this->RobotPose = pose; }

    double getError() const { return this->pmError; }

    void ProcessLaserPoints(std::array<LaserPoint, 720> &LaserPoints);
    void ProcessLaserPoints(std::array<LaserPoint, 720> &LaserPoints, const Pose &previousPose, const Pose &currentPose);

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
};

#endif // PERFECTMATCH_H