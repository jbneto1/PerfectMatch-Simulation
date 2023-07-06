#ifndef MAP_H
#define MAP_H

#include <vector>
#include <cmath>
#include <stdexcept>
#include "config.h"
#include "stb/stb_image.h"
#include "stb/stb_image_write.h"
#include "Logger/logger.h"
#include <sstream>
#include <algorithm>

class Map {
public:
    Map(Logger &logger);

    int getWidth() const { return ImgWidth; }
    int getHeight() const { return ImgHeight; }
    int getDistance(int x, int y) const { return DistMap[y][x]; }
    float getGradientX(int x, int y) const { return GradXMap[y][x]; }
    float getGradientY(int x, int y) const { return GradYMap[y][x]; }
    void setDistance(int x, int y, int value) { DistMap[y][x] = value; }
    void setGradXMap(const std::vector<std::vector<double>>& gradXMap) { GradXMap = gradXMap; }
    void setGradYMap(const std::vector<std::vector<double>>& gradYMap) { GradYMap = gradYMap; }
    void saveDistMap(const std::string& filename) const;
    void saveGradXMap(const std::string& filename, const double factor) const;
    void saveGradYMap(const std::string& filename, const double factor) const;
    void checkMaps();
    void logDistMap() const;
    void loadMaps() const;
    void loadDistMap(const std::string &filename) const;
    void loadGradXMap(const std::string &filename) const;
    void loadGradYMap(const std::string &filename) const;

private:
    int ImgWidth;
    int ImgHeight;
    std::vector<std::vector<int>> DistMap;
    std::vector<std::vector<double>> GradXMap;
    std::vector<std::vector<double>> GradYMap;
    Logger &logger;
};

#endif // MAP_H
