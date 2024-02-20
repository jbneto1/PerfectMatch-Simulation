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
    // TH_LC rotation convention is z-y'-x'' therefore Rz*Ry*Rx.
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

    u_int idx = 0;

    for (auto &point : LaserPoints)
    {

        if (point.getD() <= 0)
        {
            idx++;
            point.setIsBeamValid(false);
            continue;
        }
        // CCW rotation
        double currentAngleDegrees = degreeStep * (&point - &LaserPoints[0]);

        // Adjusted for clockwise rotation, starting from the back
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
        point.setBeamIndex(idx);

        idx++;
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

void PerfectMatch::DrawCenterAndCorners(cv::Mat &image)
{
    // Draw a black plus sign at the principal point (center of the image)
    int principal_point_x = static_cast<int>(K(0, 2));
    int principal_point_y = static_cast<int>(K(1, 2));
    int line_length = 5; // Length of the lines for the plus sign

    // Horizontal line of the plus sign
    cv::line(image,
             cv::Point(principal_point_x - line_length, principal_point_y),
             cv::Point(principal_point_x + line_length, principal_point_y),
             cv::Scalar(0, 0, 0), 2);

    // Vertical line of the plus sign
    cv::line(image,
             cv::Point(principal_point_x, principal_point_y - line_length),
             cv::Point(principal_point_x, principal_point_y + line_length),
             cv::Scalar(0, 0, 0), 2);

    // Draw black dots at the corners of the image
    int dot_radius = 5; // Radius of the dots

    // Top-left corner
    cv::circle(image, cv::Point(0, 0), dot_radius, cv::Scalar(0, 0, 0), -1);
    // Top-right corner
    cv::circle(image, cv::Point(image.cols - 1, 0), dot_radius, cv::Scalar(0, 0, 0), -1);
    // Bottom-left corner
    cv::circle(image, cv::Point(0, image.rows - 1), dot_radius, cv::Scalar(0, 0, 0), -1);
    // Bottom-right corner
    cv::circle(image, cv::Point(image.cols - 1, image.rows - 1), dot_radius, cv::Scalar(0, 0, 0), -1);
}

void PerfectMatch::DrawBoundingBox(BoundingBox &box, cv::Mat &image)
{
    // Calculate the actual top-left and bottom-right corners from the center (x, y)
    int x1 = std::clamp(static_cast<int>(box.x - box.width / 2), 0, image.cols - 1);
    int y1 = std::clamp(static_cast<int>(box.y - box.height / 2), 0, image.rows - 1);
    int x2 = std::clamp(static_cast<int>(box.x + box.width / 2), 0, image.cols - 1);
    int y2 = std::clamp(static_cast<int>(box.y + box.height / 2), 0, image.rows - 1);

    // Draw the actual bounding box in red
    cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2);

    // Calculate and draw the expanded bounding box due to SAFETY_THRESHOLD in blue
    int safety_x1 = std::max(0, x1 - SAFETY_THRESHOLD);
    int safety_y1 = std::max(0, y1 - SAFETY_THRESHOLD);
    int safety_x2 = std::min(image.cols - 1, x2 + SAFETY_THRESHOLD);
    int safety_y2 = std::min(image.rows - 1, y2 + SAFETY_THRESHOLD);

    cv::rectangle(image, cv::Point(safety_x1, safety_y1), cv::Point(safety_x2, safety_y2), cv::Scalar(255, 0, 0), 1);
}

bool PerfectMatch::isPointInsideBB(const Vector2d &point, const BoundingBox &box)
{
    if (point(0) >= (box.x - box.width / 2 - SAFETY_THRESHOLD) &&
        point(0) <= (box.x + box.width / 2 + SAFETY_THRESHOLD) &&
        point(1) >= (box.y - box.height / 2 - SAFETY_THRESHOLD) &&
        point(1) <= (box.y + box.height / 2 + SAFETY_THRESHOLD))
    {
        return true;
    }
    return false;
}

void PerfectMatch::DrawLidarPointWithAnnotation(const Vector2d &pImgPx, cv::Mat &image, u_int index, int annotateEveryN, bool isInsideBoundingBox)
{
    // Define a set of y-offsets
    std::vector<int> yOffset = {-40, -20, 20, 40};

    // Choose offset index based on the point index
    int offsetIndex = index / annotateEveryN % yOffset.size();

    // Use the chosen offset for the y position
    int yPosition = static_cast<int>(pImgPx(1)) + yOffset[offsetIndex];

    // Keep the y position within image bounds
    yPosition = std::max(0, std::min(image.rows - 1, yPosition));

    // Check if the point is within image bounds
    if (pImgPx(0) >= 0 && pImgPx(0) < image.cols && pImgPx(1) >= 0 && pImgPx(1) < image.rows)
    {

        cv::Scalar color = isInsideBoundingBox ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 127, 255); // Blue for inside, Orange for outside
        // Draw the lidar point on the image
        cv::circle(image, cv::Point(static_cast<int>(pImgPx(0)), static_cast<int>(pImgPx(1))), 3, color, -1);

        // Annotate only every nth point
        if (index % annotateEveryN == 0)
        {
            cv::putText(image, std::to_string(index), cv::Point(static_cast<int>(pImgPx(0)), yPosition),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }

        if (!isInsideBoundingBox)
        {
            logger.info("Point outside of bounding box - Index: " + std::to_string(index));
        }
    }
    else
    {
        logger.info("Point outside of image bounds - Index: " + std::to_string(index));
    }
}

void PerfectMatch::ProcessBBOutliers(std::array<LaserPoint, 720> &LaserPoints, std::vector<BoundingBox> &outliers, u_int &counter)
{
    counter = 0;
    int annotateEveryN = 5;

    cv::Mat image(480, 640, CV_8UC3);       // 480 rows x 640 columns 8 bits (0-255) 3 channels (RGB)
    image.setTo(cv::Scalar(255, 255, 255)); // Set the image to white
    DrawCenterAndCorners(image);

    // Draw all bounding boxes on the image
    for (auto &box : outliers)
    {
        DrawBoundingBox(box, image);
    }

    for (auto &point : LaserPoints)
    {
        // Transform point from lidar to camera perspective
        Vector4d pointInLidar(point.getX(), point.getY(), 0, 1); // Homogeneous coordinates
        Vector4d pointInCamera = TH_LC * pointInLidar;           // Still in homogeneous coordinates

        if (pointInCamera(2) <= 0) // If it has a negative Z it is behind the camera.
            continue;

        // Project point onto image plane
        Vector3d nullVector = Vector3d::Zero();
        MatrixXd homogeneousK(3, 4);
        homogeneousK << K, nullVector;
        Vector3d pointInImage = homogeneousK * pointInCamera; // Still in homogeneous coordinate. For euclidean, consider only u and v
        Vector2d pImgPx = (pointInImage / pointInImage(2)).head<2>();

        // Check if the point falls inside any bounding box
        bool insideAnyBoundingBox = std::any_of(outliers.begin(), outliers.end(),
                                                [&pImgPx, this](const BoundingBox &box)
                                                {
                                                    return isPointInsideBB(pImgPx, box);
                                                });

        if (insideAnyBoundingBox)
        {
            counter++;
            point.setIsBeamValid(false);
        }

        DrawLidarPointWithAnnotation(pImgPx, image, point.getBeamIndex(), annotateEveryN, insideAnyBoundingBox);
    }
    cv::imshow("Bounding Boxes and Lidar Points", image);
    int key = cv::waitKey(5) & 0xFF;
    if (key == 27)
    { // Adjusted for observed codes
        exit(0);
    }
}
