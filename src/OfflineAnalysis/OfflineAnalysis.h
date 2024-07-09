
#ifndef AMR_PROJECT_OFFLINE_ANALYSIS_H
#define AMR_PROJECT_OFFLINE_ANALYSIS_H

#include <string>
#include <fstream>
#include <array>
#include <sstream>
#include <vector>
#include <optional>
#include <iostream>
#include <chrono>
#include <thread>
#include <set>
#include <map>
#include <tuple>
#include <stdexcept>
#include "Localization/Localization.h"
#include "Logger/logger.h"
#include "data_structures/data_structures.h"
#include "Visualizer/Visualizer.h"
#include <chrono>

class OfflineAnalysis
{
public:
    OfflineAnalysis(Localization &localization, Localization &localization_w_semantics, Logger &logger, Visualizer &visualizer);
    void processLogFile(const std::string &filePath);
    // static std::tuple<std::array<int, 4>, Pose, std::optional<std::vector<LaserPoint>>, std::optional<std::vector<BoundingBox>>, long long> extractDataFromLine(const std::string &line, Logger &logger);

private:
    Localization &localization, &localization_w_semantics;
    Logger &logger;
    Visualizer &vis;

    std::tuple<std::array<int, 4>, Pose, std::optional<std::vector<LaserPoint>>,
               std::map<std::string, std::optional<std::vector<BoundingBox>>>, long long>
    extractDataFromLine(const std::string &line);
    void parseLine(const std::string &line);

    // Analysis members
    Pose EKF_pose;
    double error_PM;
    Pose error_EKF;
    Pose PM_pose;
    Matrix3d EKF_cov;

    Pose EKF_pose_semantics;
    double error_PM_semantics;
    Pose error_EKF_semantics;
    Pose PM_pose_semantics;
    Matrix3d EKF_cov_semantics;

    double time;
};

#endif // AMR_PROJECT_OFFLINE_ANALYSIS_H