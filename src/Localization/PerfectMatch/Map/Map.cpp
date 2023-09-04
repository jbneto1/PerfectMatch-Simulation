#include "Map.h"

Map::Map(Logger &logger) : logger(logger) {

    auto path_distMap = std::string(R"(../srcPython/map/distance_map.csv)");
    auto path_gradX = std::string(R"(../srcPython/map/m_estimator_gradient_x.csv)");  // Modify to the actual path if different
    auto path_gradY = std::string(R"(../srcPython/map/m_estimator_gradient_y.csv)");  // Modify to the actual path if different

    DistMap = readCSV(path_distMap);
    if (DistMap.empty()) {
        logger.error("Failed to load dist map from CSV.");
        throw std::runtime_error("Map - Failed to load dist map from CSV.");
    }

    GradXMap = readCSV(path_gradX);
    if (GradXMap.empty() || GradXMap.size() != DistMap.size() || GradXMap[0].size() != DistMap[0].size()) {
        logger.error("Failed to load GradX map from CSV with matching dimensions.");
        throw std::runtime_error("Map - Failed to load GradX map from CSV with matching dimensions.");
    }

    GradYMap = readCSV(path_gradY);
    if (GradYMap.empty() || GradYMap.size() != DistMap.size() || GradYMap[0].size() != DistMap[0].size()) {
        logger.error("Failed to load GradY map from CSV with matching dimensions.");
        throw std::runtime_error("Map - Failed to load GradY map from CSV with matching dimensions.");
    }

    ImgWidth = DistMap[0].size();
    ImgHeight = DistMap.size();

    logger.trace("All csvs read successfully.");
//
//    writeMapToCSV(DistMap, "DistMap.csv");
//    writeMapToCSV(GradXMap, "GradXMap.csv");
//    writeMapToCSV(GradYMap, "GradYMap.csv");

}

std::vector<std::vector<double>> Map::readCSV(const std::string &filePath) {
    std::vector<std::vector<double>> matrix;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        logger.error("Failed to open CSV file: " + filePath);
        throw std::runtime_error("Failed to open CSV file: " + filePath);
    }
    std::string line, value;
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value));
        }
        matrix.push_back(row);
    }
    return matrix;
}

void Map::writeMapToCSV(const std::vector<std::vector<double>> &map, const std::string &fileName) {
    std::ofstream file(fileName);

    // Ensure the file was opened successfully
    if (!file.is_open()) {
        logger.error("Unable to open file: " + fileName);
        throw std::runtime_error("Map - Unable to open file: " + fileName);
    }

    for (const auto &row: map) {
        for (const auto &element: row) {
            file << element << ',';
        }
        file << '\n';
    }

    file.close();
}
