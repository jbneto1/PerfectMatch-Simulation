#include "PerfectMatch.h"

PerfectMatch::PerfectMatch(Logger &logger, const Pose startPose, const double stepScale) : logger(
                                                                                               logger),
                                                                                           map(logger),

                                                                                           RobotPose(
                                                                                               startPose),
                                                                                           stepScale(
                                                                                               stepScale),
                                                                                           t_LC(0, -0.055, -0.155 / 2),
                                                                                           Rx(Eigen::AngleAxisd(roll, Vector3d::UnitX())),
                                                                                           Ry(Eigen::AngleAxisd(pitch, Vector3d::UnitY())),
                                                                                           Rz(Eigen::AngleAxisd(yaw, Vector3d::UnitZ())),
                                                                                           TH_LC((Matrix4d() << ((Rz.toRotationMatrix() * Ry.toRotationMatrix()) * Rx.toRotationMatrix()), t_LC,
                                                                                                  0, 0, 0, 1)
                                                                                                     .finished()),
                                                                                           K((Matrix3d() << FX, 0, CX,
                                                                                              0, FY, CY,
                                                                                              0, 0, 1)
                                                                                                 .finished()),
                                                                                           distCoeffs((VectorXd(5) << -4.24918902e-03,
                                                                                                       3.99664887e-03,
                                                                                                       2.37389148e-04,
                                                                                                       -6.17424434e-05,
                                                                                                       -1.10922867e-03)
                                                                                                          .finished()),
                                                                                           safety_threshold(SAFETY_THRESHOLD)
{
    // TH_LC rotation convention is z-y'-x'' therefore Rz*Ry*Rx.
    logger.debug(
        "Parameters: startPose (" + std::to_string(startPose.getX()) + ", " + std::to_string(startPose.getY()) +
        ", " + std::to_string(startPose.getTheta()) + "), " + ", stepScale: " +
        std::to_string(stepScale));
    meterToPixel = map.getWidth() / 1.68;
    std::cout << TH_LC << std::endl;
    pmError = 0;
}

Pose PerfectMatch::match(std::vector<LaserPoint> &data)
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

void PerfectMatch::calibrate_lidar_points(double &rx, double &ry, double px, double py)
{
    // since both frames are aligned and these lidar points are in the lidar body frame

    rx = px + OFFSET_X;
    ry = py + OFFSET_Y;

    // Lidar is behind in x and y.
}

int PerfectMatch::XTopixel(double x)
{
    return static_cast<int>(std::round(x * meterToPixel) + map.getWidth() / 2);
}

int PerfectMatch::YTopixel(double y)
{
    return static_cast<int>(std::round(-y * meterToPixel) + map.getHeight() / 2);
}

void PerfectMatch::IterLaser(std::vector<LaserPoint> &LaserPoints)
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

        // NOT WORKING
        double rx, ry;
        calibrate_lidar_points(rx, ry, laserPoint.getX(), laserPoint.getY());
        double r_cx, r_cy; // calibrated lidar world points (robots center)
        RotateAndTranslate(r_cx, r_cy, rx, ry, RobotPose.getX(), RobotPose.getY(), st, ct);

        int u = XTopixel(r_cx);
        int v = YTopixel(r_cy);

        // WORKING
        // double rx, ry;
        // RotateAndTranslate(rx, ry, laserPoint.getX(), laserPoint.getY(), RobotPose.getX(), RobotPose.getY(), st, ct);
        // int u = XTopixel(rx);
        // int v = YTopixel(ry);

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

void PerfectMatch::ProcessLaserPoints(std::vector<LaserPoint> &LaserPoints)
{
    auto start = std::chrono::high_resolution_clock::now();

    const double degreeStep = 360.0 / LaserPoints.size();
    unsigned int idx = 0;

    for (auto &point : LaserPoints)
    {
        logger.debug("Current idx: " + std::to_string(idx));

        if (point.getD() <= 0)
        {
            point.setIsBeamValid(false);
            point.setDraw(false);
            point.setBeamIndex(idx);
            idx++;
            continue;
        }
        // CCW rotation
        double currentAngleDegrees = degreeStep * idx;
        logger.debug("Current angle: " + std::to_string(currentAngleDegrees));

        double angleRadians = degToRad(currentAngleDegrees);
        angleRadians = normalizeAngle(angleRadians);
        point.setAngle(angleRadians);

        // calculate the x and y positions
        double d = point.getD();
        double x = d * cos(angleRadians);
        double y = d * sin(angleRadians);

        // set the x, y positions and std_dev for each LaserPoint
        point.setX(x);
        point.setY(y);
        point.setStdDev(1.0); // set the std_dev to 1 for now
        point.setBeamIndex(idx);

        idx++;
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    logger.trace("ProcessLaserPoints [us]: " + std::to_string(duration.count()));
}

// ------------------------ DEPRECATED -------------------------------------------------------------------------//
// TODO: need laser beam timestamps. maybe
/*
Pose PerfectMatch::interpolatePose(const Pose &previousPose, const Pose &currentPose, const double alpha)
{
    double x = previousPose.getX() + alpha * (currentPose.getX() - previousPose.getX());
    double y = previousPose.getY() + alpha * (currentPose.getY() - previousPose.getY());
    double theta = previousPose.getTheta() + alpha * (currentPose.getTheta() - previousPose.getTheta());
    return Pose(x, y, theta);
}

void PerfectMatch::ProcessLaserPoints(std::vector<LaserPoint> &LaserPoints, const Pose &previousPose, const Pose &currentPose)
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
*/
// --------------------------------------------------------------------------------------------------------------//

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

// ------------------------------------------------------------------------------------------------------------//

bool PerfectMatch::isPointInsideBB(const Vector2d &point, const BoundingBox &box)
{
    if (point(0) >= (box.x - box.width / 2 - safety_threshold) &&
        point(0) <= (box.x + box.width / 2 + safety_threshold) &&
        point(1) >= (box.y - box.height / 2 - safety_threshold) &&
        point(1) <= (box.y + box.height / 2 + safety_threshold))
    {
        return true;
    }
    return false;
}

void PerfectMatch::ProcessBBOutliersFront(std::vector<LaserPoint> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter)

{
    counter = 0;

    for (auto &point : LaserPoints)
    {
        if (!point.getIsBeamValid()) // skip invalid beams
            continue;

        // Transform point from lidar to camera perspective
        Vector4d pointInLidar(point.getX(), point.getY(), 0, 1); // Homogeneous coordinates
        Vector4d pointInCamera = TH_LC * pointInLidar;           // Still in homogeneous coordinates
        /*FIXME: name wrong. It is actually T_CL where t_cl is the offset of the lidar frame from perspective of camera frame
         and R_CL is lidar frame orientation relative from camera frame
         R_cl was assembled using roll-pitch'-yaw'' alibi convention (RzRyRx)

        */

        if (pointInCamera(2) <= 0)
        {
            point.setDraw(false);
            continue;
        } // If it has a negative Z it is behind the camera.

        // Project point onto image plane
        Vector3d nullVector = Vector3d::Zero();
        MatrixXd homogeneousK(3, 4);
        homogeneousK << K, nullVector;
        Vector3d pointInImage = homogeneousK * pointInCamera; // Still in homogeneous coordinate. For euclidean, consider only u and v
        Vector2d pImgPx = (pointInImage / pointInImage(2)).head<2>();
        point.setImgPts(pImgPx);

        // Check if the point falls inside any bounding box
        bool insideAnyBoundingBox = false;
        if (!outliers.empty())
        {
            insideAnyBoundingBox = std::any_of(outliers.begin(), outliers.end(),
                                               [&pImgPx, this](const BoundingBox &box)
                                               {
                                                   return isPointInsideBB(pImgPx, box);
                                               });
        }

        if (insideAnyBoundingBox)
        {
            counter++;
            point.setIsBeamValid(false);
        }
    }
}

// TODO: Rear, left, Right adjust transformations after fixating the cameras
void PerfectMatch::ProcessBBOutliersRear(std::vector<LaserPoint> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter)
{
    counter = 0;

    for (auto &point : LaserPoints)
    {
        if (!point.getIsBeamValid()) // skip invalid beams
            continue;

        // Transform point from lidar to camera perspective
        Vector4d pointInLidar(point.getX(), point.getY(), 0, 1); // Homogeneous coordinates
        Vector4d pointInCamera = TH_LC * pointInLidar;           // Still in homogeneous coordinates

        if (pointInCamera(2) <= 0)
        {
            point.setDraw(false);
            continue;
        } // If it has a negative Z it is behind the camera.

        // Project point onto image plane
        Vector3d nullVector = Vector3d::Zero();
        MatrixXd homogeneousK(3, 4);
        homogeneousK << K, nullVector;
        Vector3d pointInImage = homogeneousK * pointInCamera; // Still in homogeneous coordinate. For euclidean, consider only u and v
        Vector2d pImgPx = (pointInImage / pointInImage(2)).head<2>();
        point.setImgPts(pImgPx);

        // Check if the point falls inside any bounding box
        bool insideAnyBoundingBox = false;
        if (!outliers.empty())
        {
            insideAnyBoundingBox = std::any_of(outliers.begin(), outliers.end(),
                                               [&pImgPx, this](const BoundingBox &box)
                                               {
                                                   return isPointInsideBB(pImgPx, box);
                                               });
        }

        if (insideAnyBoundingBox)
        {
            counter++;
            point.setIsBeamValid(false);
        }
    }
}
void PerfectMatch::ProcessBBOutliersLeft(std::vector<LaserPoint> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter)
{
    counter = 0;

    for (auto &point : LaserPoints)
    {
        if (!point.getIsBeamValid()) // skip invalid beams
            continue;

        // Transform point from lidar to camera perspective
        Vector4d pointInLidar(point.getX(), point.getY(), 0, 1); // Homogeneous coordinates
        Vector4d pointInCamera = TH_LC * pointInLidar;           // Still in homogeneous coordinates

        if (pointInCamera(2) <= 0)
        {
            point.setDraw(false);
            continue;
        } // If it has a negative Z it is behind the camera.

        // Project point onto image plane
        Vector3d nullVector = Vector3d::Zero();
        MatrixXd homogeneousK(3, 4);
        homogeneousK << K, nullVector;
        Vector3d pointInImage = homogeneousK * pointInCamera; // Still in homogeneous coordinate. For euclidean, consider only u and v
        Vector2d pImgPx = (pointInImage / pointInImage(2)).head<2>();
        point.setImgPts(pImgPx);

        // Check if the point falls inside any bounding box
        bool insideAnyBoundingBox = false;
        if (!outliers.empty())
        {
            insideAnyBoundingBox = std::any_of(outliers.begin(), outliers.end(),
                                               [&pImgPx, this](const BoundingBox &box)
                                               {
                                                   return isPointInsideBB(pImgPx, box);
                                               });
        }

        if (insideAnyBoundingBox)
        {
            counter++;
            point.setIsBeamValid(false);
        }
    }
}
void PerfectMatch::ProcessBBOutliersRight(std::vector<LaserPoint> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter)
{
    counter = 0;

    for (auto &point : LaserPoints)
    {
        if (!point.getIsBeamValid()) // skip invalid beams
            continue;

        // Transform point from lidar to camera perspective
        Vector4d pointInLidar(point.getX(), point.getY(), 0, 1); // Homogeneous coordinates
        Vector4d pointInCamera = TH_LC * pointInLidar;           // Still in homogeneous coordinates

        if (pointInCamera(2) <= 0)
        {
            point.setDraw(false);
            continue;
        } // If it has a negative Z it is behind the camera.

        // Project point onto image plane
        Vector3d nullVector = Vector3d::Zero();
        MatrixXd homogeneousK(3, 4);
        homogeneousK << K, nullVector;
        Vector3d pointInImage = homogeneousK * pointInCamera; // Still in homogeneous coordinate. For euclidean, consider only u and v
        Vector2d pImgPx = (pointInImage / pointInImage(2)).head<2>();
        point.setImgPts(pImgPx);

        // Check if the point falls inside any bounding box
        bool insideAnyBoundingBox = false;
        if (!outliers.empty())
        {
            insideAnyBoundingBox = std::any_of(outliers.begin(), outliers.end(),
                                               [&pImgPx, this](const BoundingBox &box)
                                               {
                                                   return isPointInsideBB(pImgPx, box);
                                               });
        }

        if (insideAnyBoundingBox)
        {
            counter++;
            point.setIsBeamValid(false);
        }
    }
}

/*Optimizations*/

// TODO: results are not the same, it seems not all points are rejected
/*
void PerfectMatch::ProcessBBOutliers(std::vector<LaserPoint> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter)
{
    counter = 0;

    // First, compute the image points for all LaserPoints.
    for (auto &point : LaserPoints)
    {
        Vector4d pointInLidar(point.getX(), point.getY(), 0, 1);
        Vector4d pointInCamera = TH_LC * pointInLidar;

        // Reject points that are behind the camera or too close to it (z <= 0)
        if (pointInCamera(2) <= 0)
        {
            point.setDraw(false); // Optionally mark point as not to be drawn
            continue;
        }

        Vector3d nullVector = Vector3d::Zero();
        MatrixXd homogeneousK(3, 4);
        homogeneousK << K, nullVector; // Setup for projecting point onto image plane
        Vector3d pointInImage = homogeneousK * pointInCamera;
        Vector2d pImgPx = (pointInImage / pointInImage(2)).head<2>();
        point.setImgPts(pImgPx); // Compute and set the image coordinates
    }

    if (outliers.empty()) return;

    // Then, check each LaserPoint against the bounding boxes.
    for (const BoundingBox &box : outliers)
    {
        bool hasStartedFallingInside = false;

        for (auto &point : LaserPoints)
        {
            // Skip already invalidated points
            if (!point.getIsBeamValid())
                continue;

            // Now we only need to check if it's within the bounding box
            if (isPointInsideBB(point.getImgPts(), box))
            {
                hasStartedFallingInside = true;
                point.setIsBeamValid(false); // Invalidate the point
                counter++;
            }
            else if (hasStartedFallingInside)
            {
                // If a point was inside the bounding box, but this one isn't,
                // subsequent points are unlikely to be inside this bounding box.
                break;
            }
        }
    }
}
*/
