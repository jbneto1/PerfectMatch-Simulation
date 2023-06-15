#include "Map.h"

Map::Map(Logger &logger, const std::string &filename) : logger(logger) {
    logger.info("Loading image: " + filename);
    int channels;
    unsigned char *img = stbi_load(filename.c_str(), &ImgWidth, &ImgHeight, &channels, 0);
    if (!img) {
        logger.error("Failed to load image: " + filename); // Add log
        throw std::runtime_error("Map - Failed to load image: " + filename);
    }
    logger.trace("Image loaded successfully."); // Add log

    // Initialize the distance map and gradient maps
    DistMap = std::vector<std::vector<int>>(ImgHeight, std::vector<int>(ImgWidth, 0));
    GradXMap = std::vector<std::vector<float>>(ImgHeight, std::vector<float>(ImgWidth, 0));
    GradYMap = std::vector<std::vector<float>>(ImgHeight, std::vector<float>(ImgWidth, 0));

    // Process the image data and populate the distance map
    logger.trace("Processing image data."); // Add log
    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            int pixelValue = img[y * ImgWidth + x];
            DistMap[y][x] = pixelValue;
        }
    }
    logger.debug("Image data processed with file: " + filename); // Add log
    stbi_image_free(img);
}


void Map::saveDistMap(const std::string &filename) const {
    logger.info("Saving distance map to: " + filename);
    std::vector<unsigned char> imgData(ImgWidth * ImgHeight * 3, 0); // 3 for RGB channels

    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            int pixelValue = std::clamp(DistMap[y][x], 0, 255);
            // Assign to red, green, and blue channels
            imgData[(y * ImgWidth + x) * 3 + 0] = static_cast<unsigned char>(pixelValue); // Red
            imgData[(y * ImgWidth + x) * 3 + 1] = static_cast<unsigned char>(pixelValue); // Green
            imgData[(y * ImgWidth + x) * 3 + 2] = static_cast<unsigned char>(pixelValue); // Blue
        }
    }

    stbi_write_png(filename.c_str(), ImgWidth, ImgHeight, 3, imgData.data(), ImgWidth * 3);
    logger.debug("Successfully saved distance map to: " + filename);
}


void Map::saveGradMap(const std::string &filename, const double factor) const {
    logger.info("Saving gradient map to: " + filename);
    std::vector<unsigned char> imgData(ImgWidth * ImgHeight, 0);

    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            int pixelValue = 128 + static_cast<int>(GradXMap[y][x] * factor);
            imgData[y * ImgWidth + x] = static_cast<unsigned char>(std::clamp(pixelValue, 0, 255));
        }
    }

    stbi_write_png(filename.c_str(), ImgWidth, ImgHeight, 1, imgData.data(), ImgWidth);
    logger.debug("Successfully saved gradient map to: " + filename);
}

void Map::logDistMap() const {
    std::stringstream ss;
    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            ss << DistMap[y][x] << " ";
        }
        ss << "\n";
    }
    logger.debug(ss.str());
}
