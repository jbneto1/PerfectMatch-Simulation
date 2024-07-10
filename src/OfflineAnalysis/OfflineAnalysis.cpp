#include "OfflineAnalysis.h"

OfflineAnalysis::OfflineAnalysis(Localization &localization, Localization &localization_w_semantics, Logger &logger, Visualizer &visualizer)
    : localization(localization), localization_w_semantics(localization_w_semantics), logger(logger), vis(visualizer) {}

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
    try
    {
        std::array<int, 4> encoders;
        Pose GT_pose;
        std::optional<std::vector<LaserPoint>> optLaserReadings;
        std::map<std::string, std::optional<std::vector<BoundingBox>>> yoloDataMap;
        int timestamp;

        try
        {
            std::tie(encoders, GT_pose, optLaserReadings, yoloDataMap, timestamp) = extractDataFromLine(line);
        }
        catch (const std::exception &e)
        {
            logger.warn("Datagram empty or corrupted. Parsing failed. Exception: " + std::string(e.what()));
            return;
        }

        std::optional<std::vector<LaserPoint>> optLaserReadings_semantics;
        std::array<int, 4UL> encoders_semantics;
        Pose GT_pose_semantics;
        u_int counter = 0;

        GT_pose = localization.extrinsic_calibrate_GT(GT_pose);

        encoders_semantics = encoders;
        GT_pose_semantics = GT_pose;
        if (optLaserReadings.has_value())
            optLaserReadings_semantics = *optLaserReadings;
        else
            logger.error("parseLine: No lidar data to process!.");

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

        try
        {
            // Iterate through std::map
            for (auto &[key, val] : yoloDataMap)
            {
                if (val)
                    logger.debug("Key: " + key + ". Number of BBs: " + std::to_string(val->size()));
                else
                    logger.debug("Key: " + key + ". No bounding boxes.");

                std::vector<BoundingBox> bboxVector = val.value_or(std::vector<BoundingBox>());

                if (key == FRONTCAM)
                {
                    localization_w_semantics.getPM().ProcessBBOutliersFront(optLaserReadings_semantics.value(), bboxVector, counter);
                }
                else if (key == BACKCAM)
                {
                    localization_w_semantics.getPM().ProcessBBOutliersBack(optLaserReadings_semantics.value(), bboxVector, counter);
                }
                else if (key == LEFTCAM)
                {
                    localization_w_semantics.getPM().ProcessBBOutliersLeft(optLaserReadings_semantics.value(), bboxVector, counter);
                }
                else if (key == RIGHTCAM)
                {
                    localization_w_semantics.getPM().ProcessBBOutliersRight(optLaserReadings_semantics.value(), bboxVector, counter);
                }
            }
        }
        catch (std::exception &e)
        {
            logger.warn("Exception caught processing outliers: " + std::string(e.what()));
            exit(EXIT_FAILURE);
        }

        // With semantic interpretation
        localization_w_semantics.processData_w_PM(encoders_semantics, GT_pose_semantics, optLaserReadings_semantics.value());
        EKF_pose_semantics = localization_w_semantics.getPose();
        PM_pose_semantics = localization_w_semantics.getPM().getPose();
        error_EKF_semantics = EKF_pose_semantics - GT_pose_semantics;
        error_PM_semantics = localization_w_semantics.getPM().getError();
        EKF_cov_semantics = localization_w_semantics.getEKF().getPk();

        auto offlineData_semantics = std::make_tuple(EKF_pose_semantics, PM_pose_semantics, error_EKF_semantics, error_PM_semantics, EKF_cov_semantics, counter);

        // Log the data
        logger.fileLog_offlineAnalysis(offlineData, offlineData_semantics);

        vis.renderMultiCamViews(yoloDataMap, optLaserReadings_semantics.value());
    }
    catch (std::exception &e)
    {
        logger.error("Exception caught in parseLine method: " + std::string(e.what()));
        return;
    }
}

std::tuple<std::array<int, 4>, Pose, std::optional<std::vector<LaserPoint>>, std::map<std::string, std::optional<std::vector<BoundingBox>>>, long long>
OfflineAnalysis::extractDataFromLine(const std::string &line)
{
    std::istringstream iss(line);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(iss, token, ','))
    {
        tokens.push_back(token);
    }

    // Minimum tokens: 3 Pose + 4 Encoders + 1 Lidar (at least) + 1 Timestamp
    if (tokens.size() < 9)
    {
        throw std::runtime_error("Datagram empty or corrupted. Parsing failed.");
    }

    try
    {
        Pose pose(std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2]));
        std::array<int, 4> encoders = {std::stoi(tokens[3]), std::stoi(tokens[4]), std::stoi(tokens[5]), std::stoi(tokens[6])};
        std::vector<LaserPoint> lidarPoints;

        int currentIndex = 7; // Start of lidar data
        unsigned int count = 0;
        // Parse lidar points until a camera label or the last token (timestamp)
        std::set<std::string> cameraLabels = {"FrontCam", "RearCam", "RightCam", "LeftCam"};
        while (currentIndex < tokens.size() - 1 && cameraLabels.find(tokens[currentIndex]) == cameraLabels.end())
        {
            count++;
            LaserPoint tmp;
            tmp.setD(std::stod(tokens[currentIndex++]));
            lidarPoints.push_back(tmp);
        }
        logger.trace("Parsed " + std::to_string(count) + " laser beams.");
        std::map<std::string, std::optional<std::vector<BoundingBox>>> cameraBoundingBoxes;

        // Parse bounding boxes with camera labels
        while (currentIndex < tokens.size() - 1)
        { // Excluding the last token (timestamp)
            std::string cameraLabel = tokens[currentIndex++];
            std::vector<BoundingBox> boundingBoxes;

            if (tokens[currentIndex] == "N")
            {
                int bboxCount = std::stoi(tokens[++currentIndex]); // Read count after 'N'
                currentIndex++;                                    // Move to the start of bounding box data
                for (int i = 0; i < bboxCount; ++i)
                {
                    // Ensure there are enough tokens left for bounding box data without including the last token (timestamp)
                    if (currentIndex + 5 >= (tokens.size() - 1))
                    {
                        throw std::runtime_error("Insufficient tokens for bounding box data.");
                    }

                    int class_id = std::stoi(tokens[currentIndex++].substr(1));
                    double conf = std::stod(tokens[currentIndex++]);
                    double x = std::stod(tokens[currentIndex++]);
                    double y = std::stod(tokens[currentIndex++]);
                    double width = std::stod(tokens[currentIndex++]);
                    double height = std::stod(tokens[currentIndex++]);

                    boundingBoxes.push_back(BoundingBox{class_id, conf, x, y, width, height});
                }
                cameraBoundingBoxes[cameraLabel] = boundingBoxes;
            }
            else if (tokens[currentIndex] == "NoDetections")
            {
                cameraBoundingBoxes[cameraLabel] = std::nullopt;
                currentIndex++; // Simply skip this token
            }
            else
            {
                throw std::runtime_error("extractDataFromLine: Unexpected token while parsing camera metadata.");
            }
        }

        long long timestamp = std::stoll(tokens.back());
        static long long firstTimestamp = -1; // Normalize timestamps
        if (firstTimestamp == -1)
        {
            firstTimestamp = timestamp;
        }
        timestamp -= firstTimestamp;

        return std::make_tuple(encoders, pose, std::make_optional(lidarPoints), cameraBoundingBoxes, timestamp);
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("extractDataFromLine: Datagram corrupted. Parsing failed: " + std::string(e.what()));
    }
}