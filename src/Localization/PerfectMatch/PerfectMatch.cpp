#include "PerfectMatch.h"

PerfectMatch::PerfectMatch(Logger &logger, const Pose startPose, const double stepScale) : logger(
        logger), map(logger),
                                                                                           RobotPose(
                                                                                                   startPose),
                                                                                           stepScale(
                                                                                                   stepScale) {
    logger.debug(
            "Parameters: startPose (" + std::to_string(startPose.getX()) + ", " + std::to_string(startPose.getY()) +
            ", " + std::to_string(startPose.getTheta()) + "), "
            + ", stepScale: " +
            std::to_string(stepScale));
    meterToPixel = map.getWidth() / 1.68;
}

Pose PerfectMatch::match(std::array<LaserPoint, 720> &data) {
    // Implement the matching algorithm and return the results
    logger.trace("Running IterLaser for max iterations...");

    IterLaser(data);

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
    return static_cast<int>(std::round(x * meterToPixel) + map.getWidth() / 2);
}

int PerfectMatch::YTopixel(double y) {
    return static_cast<int>(std::round(-y * meterToPixel) + map.getHeight() / 2);
}

void PerfectMatch::IterLaser(std::array<LaserPoint, 720> &LaserPoints) {
    auto start = std::chrono::high_resolution_clock::now();
    double dx = 0;
    double dy = 0;
    double dtheta = 0;
    double st = std::sin(RobotPose.getTheta());
    double ct = std::cos(RobotPose.getTheta());
    RobotPose.setErr(0);
    int n = 0;

    for (auto &laserPoint: LaserPoints) {
        if (laserPoint.getD() < 0.1)
            continue;

        double rx, ry;
        RotateAndTranslate(rx, ry, laserPoint.getX(), laserPoint.getY(), RobotPose.getX(), RobotPose.getY(), st, ct);

        int u = XTopixel(rx);
        int v = YTopixel(ry);

        if (u >= 0 && u < map.getWidth() && v >= 0 && v < map.getHeight()) {
            double gradX = map.getGradientX(u, v);
            double gradY = map.getGradientY(u, v);

            dx -= gradX / laserPoint.getStdDev();
            dy += gradY / laserPoint.getStdDev();
            dtheta -= gradX / laserPoint.getStdDev() * (-laserPoint.getX() * st - laserPoint.getY() * ct)
                      - gradY / laserPoint.getStdDev() * (laserPoint.getX() * ct - laserPoint.getY() * st);
            laserPoint.setDx(dx);
            laserPoint.setDy(dy);
            laserPoint.setDtheta(dtheta);
            RobotPose.setErr(RobotPose.getErr() + map.getDistance(u, v));
            ++n;
        }
    }

    double adjusted_X = RobotPose.getX() + stepScale * dx;
    double adjusted_Y = RobotPose.getY() + stepScale * dy;
    double adjusted_Theta = normalizeAngle(RobotPose.getTheta() + M_PI * stepScale * dtheta);
    RobotPose.setX(adjusted_X);
    RobotPose.setY(adjusted_Y);
    RobotPose.setTheta(adjusted_Theta);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    logger.info("PerfectMatch [us]: " + std::to_string(duration.count()));
    if (n > 0) RobotPose.setErr(RobotPose.getErr() / n);
}

void PerfectMatch::ProcessLaserPoints(std::array<LaserPoint, 720> &LaserPoints) {
    auto start = std::chrono::high_resolution_clock::now();
    for (auto &point: LaserPoints) {
        if (point.getD() <= 0) continue;
        double currentAngleDegrees = degreeStep * (&point - &LaserPoints[0]);
        // convert the angle to radians
        double angleRadians = degToRad(currentAngleDegrees);

        angleRadians = normalizeAngle(angleRadians);

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
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    logger.info("ProcessLaserPoints [us]: " + std::to_string(duration.count()));
}