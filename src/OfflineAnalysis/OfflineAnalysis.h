
#ifndef AMR_PROJECT_OFFLINE_ANALYSIS_H
#define AMR_PROJECT_OFFLINE_ANALYSIS_H

#include <string>
#include <fstream>
#include <array>
#include "Localization/Localization.h"
#include "Logger/logger.h"

class OfflineAnalysis {
public:
    OfflineAnalysis(Localization& localization, Logger& logger);
    void processLogFile(const std::string& filePath);

private:
    Localization& localization;
    Logger& logger;

    void parseLine(const std::string& line);
    std::tuple<std::array<int, 4>, Pose, std::optional<std::array<LaserPoint, 720>>, std::string, long long> extractDataFromLine(const std::string& line);
};

#endif // AMR_PROJECT_OFFLINE_ANALYSIS_H