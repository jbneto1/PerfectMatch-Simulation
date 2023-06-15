#include "PerfectMatch.h"

PerfectMatch::PerfectMatch(Logger &logger, const std::string &mapFilename, const Pose startPose, const int maxIters,
                           const int cErr,
                           const double stepScale) : logger(logger), map(logger, mapFilename),
                                                     RobotPose(
                                                             startPose),
                                                     maxIters(
                                                             maxIters), c_err(cErr), stepScale(stepScale) {
    logger.info("Creating PerfectMatch with mapFilename: " + mapFilename);
    logger.debug(
            "Parameters: startPose (" + std::to_string(startPose.getX()) + ", " + std::to_string(startPose.getY()) +
            ", " + std::to_string(startPose.getTheta()) +
            "), maxIters: " + std::to_string(maxIters) + ", cErr: " + std::to_string(cErr) + ", stepScale: " +
            std::to_string(stepScale));
    PixelSize = std::max(1.7 / map.getWidth(), 1.2 / map.getHeight());
    PixelScale = 1 / PixelSize;

    // Calculate the distance and gradient maps
    logger.trace("Calculating the distance and gradient maps...");
    CalcDistMap();
    CalcGradMap();

    // Save the calculated maps
    logger.trace("Saving the calculated maps...");
    map.saveDistMap("DistMap.png");
    map.saveGradMap("GradXMap.png", 2000);
    //map.logDistMap();
    logger.debug("Successfully initialized PerfectMatch with mapFilename: " + mapFilename);
}

Pose PerfectMatch::match(std::array<LaserPoint, 720> &data) {
    // Implement the matching algorithm and return the results

    logger.trace("Processing Laser Points for matching...");
    ProcessLaserPoints(data);

    logger.trace("Running IterLaser for max iterations...");
    for (int i = 0; i <= maxIters; i++) {
        IterLaser(data);
    }

    return RobotPose;
}

void PerfectMatch::RotateAndTranslate(double &rx, double &ry, double px, double py, double tx, double ty, double st,
                                      double ct) {
    logger.trace("Rotating and translating coordinates...");
    rx = px * ct - py * st + tx;
    ry = px * st + py * ct + ty;
    logger.info("Coordinates rotated and translated.");
}

int PerfectMatch::XTopixel(double x) {
    return static_cast<int>(std::round(x * PixelScale) + map.getWidth() / 2);
}

int PerfectMatch::YTopixel(double y) {
    return static_cast<int>(std::round(-y * PixelScale) + map.getHeight() / 2);
}

void PerfectMatch::CalcDistMap() {
    int misses = 0;
    logger.trace("Calculating Distance Map...");
    for (int i = 0; i < 1000; ++i) {
        if (ScanDistMap(i) == 0) {
            ++misses;
            if (misses > 2) {
                logger.debug("misses: " + std::to_string(misses));
                break;
            }
        } else {
            misses = 0;
        }
        logger.debug("iterations: " + std::to_string(i));
    }
    logger.debug("Distance Map calculated.");
}

int PerfectMatch::ScanDistMap(int v) {
    int result = 0;
    for (int y = 1; y < map.getHeight() - 1; ++y) {
        for (int x = 1; x < map.getWidth() - 1; ++x) {
            if (map.getDistance(x, y) != v) continue;
            ++result;

            map.setDistance(x + 1, y, std::min(map.getDistance(x + 1, y), 2 + v));
            map.setDistance(x - 1, y, std::min(map.getDistance(x - 1, y), 2 + v));
            map.setDistance(x, y + 1, std::min(map.getDistance(x, y + 1), 2 + v));
            map.setDistance(x, y - 1, std::min(map.getDistance(x, y - 1), 2 + v));

            map.setDistance(x + 1, y + 1, std::min(map.getDistance(x + 1, y + 1), 3 + v));
            map.setDistance(x + 1, y - 1, std::min(map.getDistance(x + 1, y - 1), 3 + v));
            map.setDistance(x - 1, y + 1, std::min(map.getDistance(x - 1, y + 1), 3 + v));
            map.setDistance(x - 1, y - 1, std::min(map.getDistance(x - 1, y - 1), 3 + v));
        }
    }
    return result;
}

double PerfectMatch::d_err(double d) {
    double c2 = c_err * c_err;
    return 1 - c2 / (c2 + d * d);
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
    logger.info("Iteration over Laser Points complete.");
}


void PerfectMatch::CalcGradMap() {
    int width = map.getWidth();
    int height = map.getHeight();

    logger.trace("Calculating Gradient Map...");
    std::vector<std::vector<double>> GradXMap(height, std::vector<double>(width, 0));
    std::vector<std::vector<double>> GradYMap(height, std::vector<double>(width, 0));

    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            GradXMap[y][x] = (d_err(map.getDistance(x + 1, y)) - d_err(map.getDistance(x - 1, y))) / 2;
            GradYMap[y][x] = (d_err(map.getDistance(x, y + 1)) - d_err(map.getDistance(x, y - 1))) / 2;
        }
    }

    map.setGradXMap(GradXMap);
    map.setGradYMap(GradYMap);
    logger.info("Gradient Map calculated.");
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
    logger.info("Laser Points processed.");
}
