#include "OfflineAnalysis.h"

OfflineAnalysis::OfflineAnalysis(Localization &localization, Localization &localization_w_semantics, Logger &logger)
    : localization(localization), localization_w_semantics(localization_w_semantics), logger(logger) {}

void OfflineAnalysis::processLogFile(const std::string &filePath)
{
    std::ifstream logFile(filePath);
    std::string line;

    logger.info("Processing data log.");

    while (std::getline(logFile, line))
    {
        parseLine(line);
    }

    logger.info("Data log process finished.");
}

void OfflineAnalysis::parseLine(const std::string &line)
{
    auto [encoders, GT_pose, optLaserReadings, yoloData, timestamp] = extractDataFromLine(line);

    localization.getPM().ProcessLaserPoints(optLaserReadings.value());

    // Without semantic interpretation
    localization.processData_w_PM(encoders, GT_pose, optLaserReadings.value());
    EKF_pose = localization.getPose();
    PM_pose = localization.getPM().getPose();
    error_EKF = EKF_pose - GT_pose;
    error_PM = localization.getPM().getError();
    EKF_cov = localization.getEKF().getPk();

    auto offlineData = std::make_tuple(EKF_pose, PM_pose, error_EKF, error_PM, EKF_cov);

    // Process semantic interpretation
    localization_w_semantics.getPM().ProcessBBOutliers(optLaserReadings.value(), yoloData.value());

    // With semantic interpretation
    localization_w_semantics.processData_w_PM(encoders, GT_pose, optLaserReadings.value());
    EKF_pose_semantics = localization_w_semantics.getPose();
    PM_pose_semantics = localization_w_semantics.getPM().getPose();
    error_EKF_semantics = EKF_pose_semantics - GT_pose;
    error_PM_semantics = localization_w_semantics.getPM().getError();
    EKF_cov_semantics = localization_w_semantics.getEKF().getPk();

    auto offlineData_semantics = std::make_tuple(EKF_pose_semantics, PM_pose_semantics, error_EKF_semantics, error_PM_semantics, EKF_cov_semantics);

    // Log the data
    logger.fileLog_offlineAnalysis(offlineData, offlineData_semantics);

    // Update the visualizer with the new data if necessary
    // visualizer.update(GT_pose, localization.getPose(), optLaserReadings);
}

std::tuple<std::array<int, 4>, Pose, std::optional<std::array<LaserPoint, 720>>, std::optional<std::vector<BoundingBox>>, long long>
OfflineAnalysis::extractDataFromLine(const std::string &line)
{
    std::istringstream iss(line);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(iss, token, ','))
    {
        tokens.push_back(token);
    }

    if (tokens.size() < (3 /* Pose */ + 4 /* Encoders */ + 720 /* Lidar points */ + 1 /* Timestamp */))
    {
        logger.error("Datagram tokens size less than the minimum size. Returning null.");
        return std::make_tuple(std::array<int, 4>{}, Pose{}, std::nullopt, std::nullopt, 0);
    }

    try
    {
        Pose pose{std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2])};
        std::array<int, 4> encoders = {std::stoi(tokens[3]), std::stoi(tokens[4]), std::stoi(tokens[5]), std::stoi(tokens[6])};
        std::array<LaserPoint, 720> lidarPoints;

        for (int i = 0; i < 720; ++i)
        {
            lidarPoints[i].setD(std::stod(tokens[7 + i]));
        }

        // Attempt to find the first bounding box or the timestamp if no bounding boxes are present
        size_t firstBBoxOrTimestampIndex = 727; // Start index for bounding boxes or timestamp
        std::vector<BoundingBox> boundingBoxes;

        // If the first token after lidar data does not start with 'b', no bounding boxes are present
        if (tokens[firstBBoxOrTimestampIndex].rfind("b", 0) != 0)
        {
            long long timestamp = std::stoll(tokens[firstBBoxOrTimestampIndex]);
            static long long firstTimestamp = -1; // Static variable to hold the first timestamp
            if (firstTimestamp == -1)             // Check if it's the first timestamp encountered
            {
                firstTimestamp = timestamp;
            }
            timestamp -= firstTimestamp; // Subtract the first timestamp from the current timestamp
            return std::make_tuple(encoders, pose, lidarPoints, std::make_optional(boundingBoxes), timestamp);
        }

        // Parse bounding boxes
        for (size_t i = firstBBoxOrTimestampIndex; i < tokens.size() - 1; ++i)
        {
            BoundingBox box = parseBoundingBox(tokens[i]);
            boundingBoxes.push_back(box);
        }

        // Parse the timestamp, which is the last token
        long long timestamp = std::stoll(tokens.back());
        static long long firstTimestamp = -1; // Static variable to hold the first timestamp
        if (firstTimestamp == -1)             // Check if it's the first timestamp encountered
        {
            firstTimestamp = timestamp;
        }
        timestamp -= firstTimestamp; // Subtract the first timestamp from the current timestamp

        return std::make_tuple(encoders, pose, lidarPoints, std::make_optional(boundingBoxes), timestamp);
    }
    catch (const std::exception &e)
    {
        logger.error("Parsing error of the data log line. Exception: " + std::string(e.what()));
        return std::make_tuple(std::array<int, 4>{}, Pose{}, std::nullopt, std::nullopt, 0);
    }
}

BoundingBox OfflineAnalysis::parseBoundingBox(const std::string &bbox_string)
{
    std::istringstream stream(bbox_string);
    char discard;
    int id, class_id;
    double x, y, width, height;

    try
    {
        stream >> discard >> id >> discard >> class_id >> discard >> x >> discard >> y >> discard >> width >> discard >> height;
    }
    catch (const std::exception &e)
    {
        logger.error("Error parsing the bounding box string stream. Exception: " + std::string(e.what()));
        return BoundingBox{std::numeric_limits<int>::quiet_NaN(), std::numeric_limits<int>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    }

    return BoundingBox{id, class_id, x, y, width, height};
}
