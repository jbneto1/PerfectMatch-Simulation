#include "Map.h"

Map::Map(Logger &logger) : logger(logger) {
    int channels;
    int gradXWidth, gradXHeight, gradYWidth, gradYHeight;

    auto path_distMap = std::string(
            R"(../src/Localization/PerfectMatch/Map/EDTransform/dist_map_euclidean_640x480.png)");
    auto path_gradX = std::string(
            R"(../src/Localization/PerfectMatch/Map/EDTransform/grad_x_M_map_euclidean_640_480.png)");
    auto path_gradY = std::string(
            R"(../src/Localization/PerfectMatch/Map/EDTransform/grad_y_M_map_euclidean_640_480.png)");

    unsigned char *img = stbi_load(path_distMap.c_str(), &ImgWidth, &ImgHeight, &channels, 0);
    if (!img || channels != 1) {
        logger.error("Failed to load dist grayscale image.");
        throw std::runtime_error("Map - Failed to load dist grayscale image.\nLine: " + std::to_string(__LINE__) + "\nFile: " + __FILE__);
    }

    unsigned char *imgGradX = stbi_load(path_gradX.c_str(), &gradXWidth, &gradXHeight, &channels, 0);
    if (!imgGradX || channels != 1 || gradXWidth != ImgWidth || gradXHeight != ImgHeight) {
        logger.error("Failed to load grayscale GradX image with matching dimensions.");
        throw std::runtime_error("Map - Failed to load grayscale GradX image with matching dimensions.\nLine: " + std::to_string(__LINE__) + "\nFile: " + __FILE__);
    }

    unsigned char *imgGradY = stbi_load(path_gradY.c_str(), &gradYWidth, &gradYHeight, &channels, 0);
    if (!imgGradY || channels != 1 || gradYWidth != ImgWidth || gradYHeight != ImgHeight) {
        logger.error("Failed to load grayscale GradY image with matching dimensions.");
        throw std::runtime_error("Map - Failed to load grayscale GradY image with matching dimensions.\nLine: " + std::to_string(__LINE__) + "\nFile: " + __FILE__);
    }

    logger.trace("All images loaded successfully.");

    // Initialize the distance map and gradient maps
    DistMap = std::vector<std::vector<int>>(ImgHeight, std::vector<int>(ImgWidth, 0));
    GradXMap = std::vector<std::vector<double>>(ImgHeight, std::vector<double>(ImgWidth, 0.0));
    GradYMap = std::vector<std::vector<double>>(ImgHeight, std::vector<double>(ImgWidth, 0.0));

    // populate the matrices from the image data
    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            int idx = y * ImgWidth + x;

            // populate the DistMap from the main image
            DistMap[y][x] = img[idx];

            // populate the GradXMap from the gradient X image
            GradXMap[y][x] = imgGradX[idx] / 255.0;

            // populate the GradYMap from the gradient Y image
            GradYMap[y][x] = imgGradY[idx] / 255.0;
        }
    }
    //Check if the maps were loaded correctly
    checkMaps();

    stbi_image_free(img);
    stbi_image_free(imgGradX);
    stbi_image_free(imgGradY);
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

void Map::checkMaps() {
    // After populating the matrices...

// Convert the matrices back to unsigned char arrays and save them as images
    unsigned char *outputImg = new unsigned char[ImgWidth * ImgHeight];
    unsigned char *outputGradX = new unsigned char[ImgWidth * ImgHeight];
    unsigned char *outputGradY = new unsigned char[ImgWidth * ImgHeight];

    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            int idx = y * ImgWidth + x;
            outputImg[idx] = DistMap[y][x];
            outputGradX[idx] = static_cast<unsigned char>(GradXMap[y][x] * 255);
            outputGradY[idx] = static_cast<unsigned char>(GradYMap[y][x] * 255);
        }
    }

    stbi_write_png("outputDistMap.png", ImgWidth, ImgHeight, 1, outputImg, ImgWidth * sizeof(unsigned char));
    stbi_write_png("outputGradX.png", ImgWidth, ImgHeight, 1, outputGradX, ImgWidth * sizeof(unsigned char));
    stbi_write_png("outputGradY.png", ImgWidth, ImgHeight, 1, outputGradY, ImgWidth * sizeof(unsigned char));

    delete[] outputImg;
    delete[] outputGradX;
    delete[] outputGradY;
}
