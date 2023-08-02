#ifndef MAP_H
#define MAP_H

#include "third_party/stb/stb_image.h"
#include <vector>
#include <cmath>
#include <stdexcept>
#include "config/config.h"
#include "third_party/stb/stb_image_write.h"
#include "Logger/logger.h"
#include <sstream>
#include <algorithm>
#include <filesystem>

class Map {
public:
    Map(Logger &logger);

    int getWidth() const { return ImgWidth; }
    int getHeight() const { return ImgHeight; }
    int getDistance(int x, int y) const { return DistMap[y][x]; }
    float getGradientX(int x, int y) const { return GradXMap[y][x]; }
    float getGradientY(int x, int y) const { return GradYMap[y][x]; }
    void checkMaps();

private:
    int ImgWidth;
    int ImgHeight;
    std::vector<std::vector<int>> DistMap;
    std::vector<std::vector<double>> GradXMap;
    std::vector<std::vector<double>> GradYMap;
    Logger &logger;
};

#endif // MAP_H
