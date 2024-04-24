#include <fstream>
#include <iostream>
#include <string>
#include <optional>
#include <vector>
#include <array>
#include "OfflineAnalysis/OfflineAnalysis.h"
#include "data_structures/data_structures.h"

// Include your existing OfflineAnalysis class definitions or make them accessible here

std::tuple<std::array<int, 4>, Pose, std::optional<std::array<LaserPoint, 720>>, std::optional<std::vector<BoundingBox>>, long long>
extractDataFromLineArray(const std::string &line, Logger &logger)
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

// Define a function to format your data types for output (implement based on your data structures)
std::string formatOutputArray(const std::tuple<std::array<int, 4>, Pose, std::optional<std::array<LaserPoint, 720>>, std::optional<std::vector<BoundingBox>>, long long> &data, Logger &logger)
{
    std::ostringstream oss;
    auto &[encs, GT_pose, laserReadings, boundingBoxes, timestamp] = data;

    oss << GT_pose.getX() << ',' << GT_pose.getY() << ',' << GT_pose.getTheta() << ',';

    oss << encs[0] << ',' << encs[1] << ',' << encs[2] << ',' << encs[3];

    if (laserReadings.has_value())
    {
        for (const auto &reading : laserReadings.value())
        {
            oss << ',' << reading.getD();
        }
    }

    oss << ',';

    if (boundingBoxes.has_value() && !boundingBoxes->empty())
    {
        oss << "N," << boundingBoxes->size();
        for (const auto &box : boundingBoxes.value())
        {
            oss << ",b0" << ',' << box.conf << ',' << box.x << ',' << box.y << ',' << box.width << ',' << box.height;
            logger.debug("Box.conf: " + std::to_string(box.conf) + ". Box.x: " + std::to_string(box.x) + ". Box.y: " + std::to_string(box.y) + ". Box.width: " + std::to_string(box.width) + ". Box.height: " + std::to_string(box.height));
        }
    }
    else
    {
        if (!boundingBoxes.has_value())
        {
            logger.debug("Bounding boxes are not present.");
        }
        else if (boundingBoxes->empty())
        {
            logger.debug("Bounding boxes are present but the vector is empty.");
        }
        oss << "NoDetections";
    }

    oss << ',' << timestamp;

    logger.debug("Timestamp: " + std::to_string(timestamp));

    return oss.str();
}
std::string formatOutput(const std::tuple<std::array<int, 4>, Pose, std::optional<std::vector<LaserPoint>>, std::optional<std::vector<BoundingBox>>, long long> &data, Logger &logger)
{
    std::ostringstream oss;
    auto &[encs, GT_pose, laserReadings, boundingBoxes, timestamp] = data;

    oss << GT_pose.getX() << ',' << GT_pose.getY() << ',' << GT_pose.getTheta() << ',';

    oss << encs[0] << ',' << encs[1] << ',' << encs[2] << ',' << encs[3];

    if (laserReadings.has_value())
    {
        for (const auto &reading : laserReadings.value())
        {
            oss << ',' << reading.getD();
        }
    }

    oss << ',';

    if (boundingBoxes.has_value() && !boundingBoxes->empty())
    {
        oss << "N," << boundingBoxes->size();
        for (const auto &box : boundingBoxes.value())
        {
            oss << ",b0" << ',' << box.conf << ',' << box.x << ',' << box.y << ',' << box.width << ',' << box.height;
            // logger.debug("Box.conf: " + std::to_string(box.conf) + ". Box.x: " + std::to_string(box.x) + ". Box.y: " + std::to_string(box.y) + ". Box.width: " + std::to_string(box.width) + ". Box.height: " + std::to_string(box.height));
        }
    }
    else
    {
        if (!boundingBoxes.has_value())
        {
            logger.debug("Bounding boxes are not present.");
        }
        else if (boundingBoxes->empty())
        {
            logger.debug("Bounding boxes are present but the vector is empty.");
        }
        oss << "NoDetections";
    }

    oss << ',' << timestamp;

    logger.debug("Timestamp: " + std::to_string(timestamp));

    return oss.str();
}

int main()
{
    Logger &logger = Logger::getInstance(spdlog::level::debug);

    std::ifstream logFile("/home/braun/Repositories/PerfectMatch-Simulation/docs/logs/container_test.txt");
    std::ofstream outFileBefore("output_before_refactor.txt");
    std::ofstream outFileAfter("output_after_refactor.txt");

    if (!outFileBefore)
    {
        logger.error("Failed to open output_before_refactor.txt for writing.");
        return 1;
    }
    if (!outFileAfter)
    {
        logger.error("Failed to open output_after_refactor.txt for writing.");
        return 1;
    }

    std::string line;
    while (std::getline(logFile, line))
    {
        // Assuming extractDataFromLineBefore and extractDataFromLineAfter are available
        auto resultBefore = extractDataFromLineArray(line, logger);
        auto resultAfter = OfflineAnalysis::extractDataFromLine(line, logger);

        outFileBefore << formatOutputArray(resultBefore, logger) << "\n";
        outFileBefore.flush();
        outFileAfter << formatOutput(resultAfter, logger) << "\n";
        outFileAfter.flush();
    }

    outFileBefore.close();
    outFileAfter.close();
    logFile.close();

    std::cout << "Comparison files generated: output_before_refactor.txt and output_after_refactor.txt" << std::endl;

    return 0;
}
