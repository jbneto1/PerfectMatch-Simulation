#include "Map.h"

Map::Map(const std::string& filename) {
    int channels;
    unsigned char* img = stbi_load(filename.c_str(), &ImgWidth, &ImgHeight, &channels, 0);
    if (!img) {
        throw std::runtime_error("Map - Failed to load image: " + filename);
    }

    // Initialize the distance map and gradient maps
    DistMap = std::vector<std::vector<int>>(ImgHeight, std::vector<int>(ImgWidth, 0));
    GradXMap = std::vector<std::vector<float>>(ImgHeight, std::vector<float>(ImgWidth, 0));
    GradYMap = std::vector<std::vector<float>>(ImgHeight, std::vector<float>(ImgWidth, 0));

    // Process the image data and populate the distance map
    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            int pixelValue = img[y * ImgWidth + x];
            DistMap[y][x] = pixelValue;
        }
    }

    stbi_image_free(img);
}

void Map::saveAsImage(const std::string& filename) const {
    std::vector<unsigned char> imgData(ImgWidth * ImgHeight, 0);

    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            imgData[y * ImgWidth + x] = static_cast<unsigned char>(DistMap[y][x]);
        }
    }

    stbi_write_png(filename.c_str(), ImgWidth, ImgHeight, 1, imgData.data(), ImgWidth);
}

void Map::saveAsImageGradX(const std::string& filename) const {
    std::vector<unsigned char> imgData(ImgWidth * ImgHeight, 0);

    float gradXMax = std::numeric_limits<float>::min();
    float gradXMin = std::numeric_limits<float>::max();

    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            gradXMax = std::max(gradXMax, std::abs(GradXMap[y][x]));
            gradXMin = std::min(gradXMin, std::abs(GradXMap[y][x]));
        }
    }

    float scale = 255.0f / gradXMax;

    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            int pixelValue = 128 + static_cast<int>(scale * GradXMap[y][x]);
            imgData[y * ImgWidth + x] = static_cast<unsigned char>(pixelValue);
        }
    }

    stbi_write_png(filename.c_str(), ImgWidth, ImgHeight, 1, imgData.data(), ImgWidth);
}

void Map::saveAsImageGradY(const std::string& filename) const {
    std::vector<unsigned char> imgData(ImgWidth * ImgHeight, 0);

    float gradYMax = std::numeric_limits<float>::min();
    float gradYMin = std::numeric_limits<float>::max();

    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            gradYMax = std::max(gradYMax, std::abs(GradYMap[y][x]));
            gradYMin = std::min(gradYMin, std::abs(GradYMap[y][x]));
        }
    }

    float scale = 255.0f / gradYMax;

    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            int pixelValue = 128 + static_cast<int>(scale * GradYMap[y][x]);
            imgData[y * ImgWidth + x] = static_cast<unsigned char>(pixelValue);
        }
    }

    stbi_write_png(filename.c_str(), ImgWidth, ImgHeight, 1, imgData.data(), ImgWidth);
}

