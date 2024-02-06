#include "PerfectMatch.h"

PerfectMatch::PerfectMatch(Logger &logger, const Pose startPose, const double stepScale) : logger(
                                                                                               logger),
                                                                                           map(logger),
                                                                                           RobotPose(
                                                                                               startPose),
                                                                                           stepScale(
                                                                                               stepScale),
                                                                                           t_LC(-0.155 / 2, 0, -0.055),
                                                                                           Rx(Matrix3d::Identity()),
                                                                                           Ry((Matrix3d() << 0, 0, -1,
                                                                                               0, 1, 0,
                                                                                               1, 0, 0)
                                                                                                  .finished()),
                                                                                           Rz((Matrix3d() << 0, -1, 0,
                                                                                               1, 0, 0,
                                                                                               0, 0, 1)
                                                                                                  .finished()),
                                                                                           TH_LC((Matrix4d() << Rz * Ry * Rx, t_LC,
                                                                                                  0, 0, 0, 1)
                                                                                                     .finished()),
                                                                                           K((Matrix3d() << 219.96470465, 0, 319.21197429,
                                                                                              0, 219.94273694, 241.81387698,
                                                                                              0, 0, 1)
                                                                                                 .finished()),
                                                                                           distCoeffs((VectorXd(5) << -4.24918902e-03,
                                                                                                       3.99664887e-03,
                                                                                                       2.37389148e-04,
                                                                                                       -6.17424434e-05,
                                                                                                       -1.10922867e-03)
                                                                                                          .finished())
{
    logger.debug(
        "Parameters: startPose (" + std::to_string(startPose.getX()) + ", " + std::to_string(startPose.getY()) +
        ", " + std::to_string(startPose.getTheta()) + "), " + ", stepScale: " +
        std::to_string(stepScale));
    meterToPixel = map.getWidth() / 1.68;
    pmError = 0;
}

Pose PerfectMatch::match(std::array<LaserPoint, 720> &data)
{
    // Implement the matching algorithm and return the results

    IterLaser(data);

    return RobotPose;
}

void PerfectMatch::RotateAndTranslate(double &rx, double &ry, double px, double py, double tx, double ty, double st,
                                      double ct)
{
    logger.trace("Rotating and translating coordinates...");
    rx = px * ct - py * st + tx;
    ry = px * st + py * ct + ty;
    logger.trace("Coordinates rotated and translated.");
}

int PerfectMatch::XTopixel(double x)
{
    return static_cast<int>(std::round(x * meterToPixel) + map.getWidth() / 2);
}

int PerfectMatch::YTopixel(double y)
{
    return static_cast<int>(std::round(-y * meterToPixel) + map.getHeight() / 2);
}

void PerfectMatch::IterLaser(std::array<LaserPoint, 720> &LaserPoints)
{
    auto start = std::chrono::high_resolution_clock::now();
    double dx = 0;
    double dy = 0;
    double dtheta = 0;
    double st = std::sin(RobotPose.getTheta());
    double ct = std::cos(RobotPose.getTheta());
    pmError = 0;
    int n = 0;

    for (auto &laserPoint : LaserPoints)
    {
        if ((laserPoint.getD() < 0.1) || (!laserPoint.getIsBeamValid()))
            continue;

        double rx, ry;
        RotateAndTranslate(rx, ry, laserPoint.getX(), laserPoint.getY(), RobotPose.getX(), RobotPose.getY(), st, ct);

        int u = XTopixel(rx);
        int v = YTopixel(ry);

        if (u >= 0 && u < map.getWidth() && v >= 0 && v < map.getHeight())
        {
            double gradX = map.getGradientX(u, v);
            double gradY = map.getGradientY(u, v);

            dx -= gradX / laserPoint.getStdDev();
            dy += gradY / laserPoint.getStdDev();
            dtheta -= gradX / laserPoint.getStdDev() * (-laserPoint.getX() * st - laserPoint.getY() * ct) - gradY / laserPoint.getStdDev() * (laserPoint.getX() * ct - laserPoint.getY() * st);
            laserPoint.setDx(dx);
            laserPoint.setDy(dy);
            laserPoint.setDtheta(dtheta);
            pmError += map.getDistance(u, v);
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
    logger.trace("Full IterLaser [us]: " + std::to_string(duration.count()));
    if (n > 0)
        pmError = pmError / n;
}

void PerfectMatch::ProcessLaserPoints(std::array<LaserPoint, 720> &LaserPoints)
{
    auto start = std::chrono::high_resolution_clock::now();
    for (auto &point : LaserPoints)
    {

        if (point.getD() <= 0)
        {
            point.setIsBeamValid(false);
            continue;
        }
        point.setIsBeamValid(true);

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
    logger.trace("ProcessLaserPoints [us]: " + std::to_string(duration.count()));
}

// ------------------------ DEPRECATED -------------------------------------------------------------------------//

Pose PerfectMatch::interpolatePose(const Pose &previousPose, const Pose &currentPose, const double alpha)
{
    double x = previousPose.getX() + alpha * (currentPose.getX() - previousPose.getX());
    double y = previousPose.getY() + alpha * (currentPose.getY() - previousPose.getY());
    double theta = previousPose.getTheta() + alpha * (currentPose.getTheta() - previousPose.getTheta());
    return Pose(x, y, theta);
}

void PerfectMatch::ProcessLaserPoints(std::array<LaserPoint, 720> &LaserPoints, const Pose &previousPose, const Pose &currentPose)
{
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 720; ++i)
    {
        double alpha = static_cast<double>(i) / 720.0;

        Pose interpolatedPose = interpolatePose(previousPose, currentPose, alpha);

        // Process each laser point with the interpolated pose
        double adjustedX, adjustedY;
        RotateAndTranslate(adjustedX, adjustedY, LaserPoints[i].getX(), LaserPoints[i].getY(),
                           interpolatedPose.getX(), interpolatedPose.getY(),
                           sin(interpolatedPose.getTheta()), cos(interpolatedPose.getTheta()));

        LaserPoints[i].setX(adjustedX);
        LaserPoints[i].setY(adjustedY);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    logger.info("ProcessLaserPoints [us]: " + std::to_string(duration.count()));
}

// --------------------------------------------------------------------------------------------------------------//

void PerfectMatch::digitalToClassicOrigin(Vector3d &point)
{
    point(0) = point(0) - K(0, 2);
    point(1) = K(1, 2) - point(1);
}

// TODO: implement the distortion correction for the pinhole camera model.

// void PerfectMatch::CorrectDistortion(Vector3d &point)
// {
//     // Normalize the point using the focal length and principal point
//     double x = (point(0) - K(0, 2)) / K(0, 0);
//     double y = (point(1) - K(1, 2)) / K(1, 1);

//     double r2 = x * x + y * y;
//     double r4 = r2 * r2;
//     double r6 = r4 * r2;

//     // Assuming distCoeffs is [k1, k2, p1, p2, k3]
//     double k1 = distCoeffs(0);
//     double k2 = distCoeffs(1);
//     double p1 = distCoeffs(2);
//     double p2 = distCoeffs(3);
//     double k3 = distCoeffs(4);

//     // Apply distortion correction
//     double xCorrected = x * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
//     double yCorrected = y * (1 + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

//     // Denormalize the corrected point to get pixel coordinates
//     point(0) = xCorrected * K(0, 0) + K(0, 2);
//     point(1) = yCorrected * K(1, 1) + K(1, 2);
// }

void PerfectMatch::ProcessBBOutliers(std::array<LaserPoint, 720> &LaserPoints, std::vector<BoundingBox> &outliers)
{
    for (auto &bbox : outliers)
    {
        bool insideBoundingBox = false;

        for (auto &point : LaserPoints)
        {
            // Transform point from lidar to camera perspective
            Eigen::Vector4d pointInLidar(point.getX(), point.getY(), 0, 1);
            Eigen::Vector4d pointInCamera = TH_LC * pointInLidar;

            // Project point onto image plane
            Eigen::Vector3d pointInImage = K * (pointInCamera / pointInCamera(2)).head<3>();

            digitalToClassicOrigin(pointInImage);

            // Correct camera distortions
            // CorrectDistortion(pointInImage);

            // Check if point falls inside BB
            if (pointInImage(0) >= (bbox.x - SAFETY_THRESHOLD) && pointInImage(0) <= (bbox.x + bbox.width + SAFETY_THRESHOLD) &&
                pointInImage(1) >= (bbox.y - SAFETY_THRESHOLD) && pointInImage(1) <= (bbox.y + bbox.height + SAFETY_THRESHOLD))
            {
                // First beam started falling inside BB
                insideBoundingBox = true;
                // Reject it
                point.setIsBeamValid(false);
            }
            else if (insideBoundingBox)
            {
                // If the point was previously inside a bounding box and now it's outside, break the loop
                break;
            }
        }
    }
}
