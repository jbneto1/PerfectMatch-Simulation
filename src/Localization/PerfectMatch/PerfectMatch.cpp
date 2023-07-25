#include "PerfectMatch.h"

PerfectMatch::PerfectMatch(Logger &logger, const Pose startPose, const int maxIters,
                           const int cErr,
                           const double stepScale) : logger(logger), map(logger),
                                                     RobotPose(
                                                             startPose),
                                                     maxIters(
                                                             maxIters), c_err(cErr), stepScale(stepScale) {
    logger.debug(
            "Parameters: startPose (" + std::to_string(startPose.getX()) + ", " + std::to_string(startPose.getY()) +
            ", " + std::to_string(startPose.getTheta()) +
            "), maxIters: " + std::to_string(maxIters) + ", cErr: " + std::to_string(cErr) + ", stepScale: " +
            std::to_string(stepScale));
    PixelSizeWidth = 1.7 / map.getWidth();
    PixelScaleWidth = 1 / PixelSizeWidth;
    PixelSizeHeight = 1.2 / map.getHeight();
    PixelScaleHeight = 1 / PixelSizeHeight;
}

Pose PerfectMatch::match(std::array<LaserPoint, 720> &data) {
    // Implement the matching algorithm and return the results

    logger.trace("Processing Laser Points for matching...");
    ProcessLaserPoints(data);

    logger.trace("Running IterLaser for max iterations...");
    for (int i = 0; i < maxIters; i++) {
        IterLaser(data);
    }

    return RobotPose;
}

void PerfectMatch::RotateAndTranslate(double &rx, double &ry, double px, double py, double tx, double ty, double st,
                                      double ct) {
    logger.trace("Rotating and translating coordinates...");
    rx = px * ct - py * st + tx;
    ry = px * st + py * ct + ty;
    logger.trace("Coordinates rotated and translated.");
}

int PerfectMatch::XTopixel(double x) {
    return static_cast<int>(std::round(x * PixelScaleWidth) + map.getWidth() / 2);
}

int PerfectMatch::YTopixel(double y) {
    return static_cast<int>(std::round(-y * PixelScaleHeight) + map.getHeight() / 2);
}

void PerfectMatch::IterLaser(const std::array<LaserPoint, 720> &LaserPoints) {
    logger.trace("Iterating over Laser Points...");
    double dx = 0;
    double dy = 0;
    double dtheta = 0;
    double st = std::sin(RobotPose.getTheta());
    double ct = std::cos(RobotPose.getTheta());
    RobotPose.setErr(0);
    int n = 0;

    for (const auto &laserPoint: LaserPoints) {
        if (laserPoint.getD() < 0.1)
            continue;

        double rx, ry;
        RotateAndTranslate(rx, ry, laserPoint.getX(), laserPoint.getY(), RobotPose.getX(), RobotPose.getY(), st, ct);
        int u = XTopixel(rx);
        int v = YTopixel(ry);

        if (u > 0 && u < map.getWidth() - 1 && v > 0 && v < map.getHeight() - 1) {
            double gradX = map.getGradientX(u, v);
            double gradY = map.getGradientY(u, v);

            dx -= gradX / laserPoint.getStdDev();
            dy += gradY / laserPoint.getStdDev();
            dtheta -= gradX / laserPoint.getStdDev() * (-laserPoint.getX() * st - laserPoint.getY() * ct)
                      + gradY / laserPoint.getStdDev() * (laserPoint.getX() * ct - laserPoint.getY() * st);
            RobotPose.setErr(RobotPose.getErr() + map.getDistance(u, v));
            ++n;
        }
    }

    RobotPose.setX(RobotPose.getX() + stepScale * dx);
    RobotPose.setY(RobotPose.getY() + stepScale * dy);
    RobotPose.setTheta(RobotPose.getTheta() + M_PI * stepScale * dtheta);
    if (n > 0) RobotPose.setErr(RobotPose.getErr() / n);
    logger.trace("Match complete.");
}

void PerfectMatch::ProcessLaserPoints(std::array<LaserPoint, 720> &LaserPoints) {
    logger.trace("Processing Laser Points...");
    for (auto &point: LaserPoints) {
        double currentAngleDegrees = degreeStep * (&point - &LaserPoints[0]);
        // convert the angle to radians
        double angleRadians = currentAngleDegrees * M_PI / 180.0;

        // set the angle for each LaserPoint
        point.setAngle(angleRadians);

        // calculate the x and y positions
        double d = point.getD();
        double x = d * cos(angleRadians);
        double y = d * sin(angleRadians);

        // set the x, y positions and std_dev for each LaserPoint
        point.setX(x);
        point.setY(y);
        point.setStdDev(1.0); // set the std_dev to 1 for now
    }
    logger.trace("Laser Points processed.");

    //TODO fix the logger msgs and check datagrams in real time to remove this hypothesis of error from the PM algorithm
    //TODO after that, check what could be causing the 90 degree offset in theta
}
