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

    std::optional<std::array<LaserPoint, 720UL>> optLaserReadings_semantics;

    std::array<int, 4UL> encoders_semantics;

    encoders_semantics = encoders;

    Pose GT_pose_semantics = GT_pose;

    optLaserReadings_semantics = *optLaserReadings;

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
    localization_w_semantics.getPM().ProcessLaserPoints(optLaserReadings_semantics.value());
    localization_w_semantics.getPM().ProcessBBOutliers(optLaserReadings_semantics.value(), yoloData.value());

    // With semantic interpretation
    localization_w_semantics.processData_w_PM(encoders_semantics, GT_pose_semantics, optLaserReadings_semantics.value());
    EKF_pose_semantics = localization_w_semantics.getPose();
    PM_pose_semantics = localization_w_semantics.getPM().getPose();
    error_EKF_semantics = EKF_pose_semantics - GT_pose_semantics;
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
        Pose pose(std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2]));
        std::array<int, 4> encoders = {std::stoi(tokens[3]), std::stoi(tokens[4]), std::stoi(tokens[5]), std::stoi(tokens[6])};
        std::array<LaserPoint, 720> lidarPoints;

        for (int i = 0; i < 720; ++i)
        {
            lidarPoints[i].setD(std::stod(tokens[7 + i]));
        }

        size_t currentIndex = 727; // Directly after lidar points
        std::vector<BoundingBox> boundingBoxes;

        try
        {
            // bool isTokenN = (tokens[currentIndex] == "N");
            if ((currentIndex < tokens.size()) && (tokens[currentIndex] == "N"))
            {
                size_t bboxCount = std::stoi(tokens[++currentIndex]);
                currentIndex++; // Move past the bounding box count

                for (size_t i = 0; i < bboxCount; ++i)
                {
                    if (currentIndex + 5 > tokens.size())
                    {
                        throw std::runtime_error("Not enough tokens for bounding box data.");
                    }

                    // Parse bounding box data
                    int class_id = std::stoi(tokens[currentIndex++].substr(1));
                    double conf = std::stod(tokens[currentIndex++]);
                    double x = std::stod(tokens[currentIndex++]);
                    double y = std::stod(tokens[currentIndex++]);
                    double width = std::stod(tokens[currentIndex++]);
                    double height = std::stod(tokens[currentIndex++]);

                    boundingBoxes.push_back(BoundingBox{class_id, conf, x, y, width, height});
                }
            }
            long long timestamp = std::stoll(tokens.back());
            static long long firstTimestamp = -1; // To normalize timestamps
            if (firstTimestamp == -1)
            {
                firstTimestamp = timestamp;
            }
            timestamp -= firstTimestamp;

            return std::make_tuple(encoders, pose, std::make_optional(lidarPoints), std::make_optional(boundingBoxes), timestamp);
        }
        catch (const std::exception &e)
        {
            logger.error("General BB parsing error of the data log line. Exception: " + std::string(e.what()));
            return std::make_tuple(std::array<int, 4>{}, Pose{}, std::nullopt, std::nullopt, 0);
        }
    }
    catch (const std::exception &e)
    {
        logger.error("General proprioceptive data parsing error of the data log line. Exception: " + std::string(e.what()));
        return std::make_tuple(std::array<int, 4>{}, Pose{}, std::nullopt, std::nullopt, 0);
    }
}