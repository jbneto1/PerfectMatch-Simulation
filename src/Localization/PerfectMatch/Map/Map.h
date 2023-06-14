#ifndef MAP_H
#define MAP_H

#include <vector>
#include <cmath>
#include <stdexcept>
#include "config.h"
#include "stb/stb_image.h"
#include "stb/stb_image_write.h"

class Map {
public:
    Map(const std::string& filename);
    void saveAsImage(const std::string& filename) const;
    int getWidth() const { return ImgWidth; }
    int getHeight() const { return ImgHeight; }
    int getDistance(int x, int y) const { return DistMap[y][x]; }
    float getGradientX(int x, int y) const { return GradXMap[y][x]; }
    float getGradientY(int x, int y) const { return GradYMap[y][x]; }
    void setDistance(int x, int y, int value) { DistMap[y][x] = value; }
    void setGradXMap(const std::vector<std::vector<float>>& gradXMap) { GradXMap = gradXMap; }
    void setGradYMap(const std::vector<std::vector<float>>& gradYMap) { GradYMap = gradYMap; }
    void saveAsImageGradY(const std::string& filename) const;
    void saveAsImageGradX(const std::string& filename) const;

private:
    int ImgWidth;
    int ImgHeight;
    std::vector<std::vector<int>> DistMap;
    std::vector<std::vector<float>> GradXMap;
    std::vector<std::vector<float>> GradYMap;
};

#endif // MAP_H
