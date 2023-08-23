#include "Map.h"

Map::Map(Logger &logger) : logger(logger) {
    int channels;
    int gradXWidth, gradXHeight, gradYWidth, gradYHeight;

    auto path_distMap = std::string(
            R"(../src/Localization/PerfectMatch/Map/EDTransform/dist_map_euclidean_800x566.png)");
    auto path_gradX = std::string(
            R"(../src/Localization/PerfectMatch/Map/EDTransform/grad_x_M_map_euclidean_800_566.png)");
    auto path_gradY = std::string(
            R"(../src/Localization/PerfectMatch/Map/EDTransform/grad_y_M_map_euclidean_800_566.png)");

    unsigned char *img = stbi_load(path_distMap.c_str(), &ImgWidth, &ImgHeight, &channels, 0);
    if (!img || channels != 1) {
        logger.error("Failed to load dist grayscale image.");
        throw std::runtime_error(
                "Map - Failed to load dist grayscale image.\nLine: " + std::to_string(__LINE__) + "\nFile: " +
                __FILE__);
    }

    unsigned char *imgGradX = stbi_load(path_gradX.c_str(), &gradXWidth, &gradXHeight, &channels, 0);
    if (!imgGradX || channels != 1 || gradXWidth != ImgWidth || gradXHeight != ImgHeight) {
        logger.error("Failed to load grayscale GradX image with matching dimensions.");
        throw std::runtime_error("Map - Failed to load grayscale GradX image with matching dimensions.\nLine: " +
                                 std::to_string(__LINE__) + "\nFile: " + __FILE__);
    }

    unsigned char *imgGradY = stbi_load(path_gradY.c_str(), &gradYWidth, &gradYHeight, &channels, 0);
    if (!imgGradY || channels != 1 || gradYWidth != ImgWidth || gradYHeight != ImgHeight) {
        logger.error("Failed to load grayscale GradY image with matching dimensions.");
        throw std::runtime_error("Map - Failed to load grayscale GradY image with matching dimensions.\nLine: " +
                                 std::to_string(__LINE__) + "\nFile: " + __FILE__);
    }

    logger.trace("All images loaded successfully.");

    // Initialize the distance map and gradient maps
    DistMap = std::vector<std::vector<double>>(ImgHeight, std::vector<double>(ImgWidth, 0.0));
    GradXMap = std::vector<std::vector<double>>(ImgHeight, std::vector<double>(ImgWidth, 0.0));
    GradYMap = std::vector<std::vector<double>>(ImgHeight, std::vector<double>(ImgWidth, 0.0));

//    // Populate the matrices from the image data
//    for (int y = 0; y < ImgHeight; ++y) {
//        for (int x = 0; x < ImgWidth; ++x) {
//            int idx = y * ImgWidth + x;
//
//            // populate the DistMap from the main image
//            DistMap[y][x] = (img[idx] / 255.0) * MAX_DIST_VALUE; //max distance value before normalization
//
//            // populate the GradXMap from the gradient X image
//            GradXMap[y][x] = ((imgGradX[idx] / 255.0) * 2 - 1) * MAX_GRAD_VALUE; // max gradient value after M est
//
//            // populate the GradYMap from the gradient Y image
//            GradYMap[y][x] = ((imgGradY[idx] / 255.0) * 2 - 1) * MAX_GRAD_VALUE; // same
//        }
//    }

    //Inverting the reading order of the images (rows)

    // Populate the matrices from the image data
    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            int idx = (ImgHeight - 1 - y) * ImgWidth + x;  // This will start reading from the last row

            // populate the DistMap from the main image
            DistMap[y][x] = (img[idx] / 255.0) * MAX_DIST_VALUE; //max distance value before normalization

            // populate the GradXMap from the gradient X image
            GradXMap[y][x] = ((imgGradX[idx] / 255.0) * 2 - 1) * MAX_GRAD_VALUE; // max gradient value after M est

            // populate the GradYMap from the gradient Y image
            GradYMap[y][x] = ((imgGradY[idx] / 255.0) * 2 - 1) * MAX_GRAD_VALUE; // same
        }
    }

    double maxDist = std::numeric_limits<double>::lowest();
    double minDist = std::numeric_limits<double>::max();
    double maxGradX = std::numeric_limits<double>::lowest();
    double minGradX = std::numeric_limits<double>::max();
    double maxGradY = std::numeric_limits<double>::lowest();
    double minGradY = std::numeric_limits<double>::max();

    for (const auto &row : DistMap)
    {
        maxDist = std::max(maxDist, *std::max_element(row.begin(), row.end()));
        minDist = std::min(minDist, *std::min_element(row.begin(), row.end()));
    }

    for (const auto &row : GradXMap)
    {
        maxGradX = std::max(maxGradX, *std::max_element(row.begin(), row.end()));
        minGradX = std::min(minGradX, *std::min_element(row.begin(), row.end()));
    }

    for (const auto &row : GradYMap)
    {
        maxGradY = std::max(maxGradY, *std::max_element(row.begin(), row.end()));
        minGradY = std::min(minGradY, *std::min_element(row.begin(), row.end()));
    }

    logger.info("Dist: max = " + std::to_string(maxDist) + ", min = " + std::to_string(minDist));
    logger.info("GradX: max = " + std::to_string(maxGradX) + ", min = " + std::to_string(minGradX));
    logger.info("GradY: max = " + std::to_string(maxGradY) + ", min = " + std::to_string(minGradY));
    //Check if the maps were loaded correctly
    checkMaps();

    stbi_image_free(img);
    stbi_image_free(imgGradX);
    stbi_image_free(imgGradY);
}

void Map::checkMaps() {
    // After populating the matrices...

// Convert the matrices back to unsigned char arrays and save them as images
    unsigned char *outputImg = new unsigned char[ImgWidth * ImgHeight];
    unsigned char *outputGradX = new unsigned char[ImgWidth * ImgHeight];
    unsigned char *outputGradY = new unsigned char[ImgWidth * ImgHeight];

//    for (int y = 0; y < ImgHeight; ++y) {
//        for (int x = 0; x < ImgWidth; ++x) {
//            int idx = y * ImgWidth + x;
//            outputImg[idx] = static_cast<unsigned char>((DistMap[y][x] / MAX_DIST_VALUE) * 255);
//            outputGradX[idx] = static_cast<unsigned char>(((GradXMap[y][x] / MAX_GRAD_VALUE + 1) / 2) * 255);
//            outputGradY[idx] = static_cast<unsigned char>(((GradYMap[y][x] / MAX_GRAD_VALUE + 1) / 2) * 255);
//        }
//    }

    for (int y = 0; y < ImgHeight; ++y) {
        for (int x = 0; x < ImgWidth; ++x) {
            int idx = (ImgHeight - 1 - y) * ImgWidth + x;  // This will start writing from the last row
            outputImg[idx] = static_cast<unsigned char>((DistMap[y][x] / MAX_DIST_VALUE) * 255);
            outputGradX[idx] = static_cast<unsigned char>(((GradXMap[y][x] / MAX_GRAD_VALUE + 1) / 2) * 255);
            outputGradY[idx] = static_cast<unsigned char>(((GradYMap[y][x] / MAX_GRAD_VALUE + 1) / 2) * 255);
        }
    }

    stbi_write_png("outputDistMap.png", ImgWidth, ImgHeight, 1, outputImg, ImgWidth * sizeof(unsigned char));
    stbi_write_png("outputGradX.png", ImgWidth, ImgHeight, 1, outputGradX, ImgWidth * sizeof(unsigned char));
    stbi_write_png("outputGradY.png", ImgWidth, ImgHeight, 1, outputGradY, ImgWidth * sizeof(unsigned char));

    //TODO CHECK THE GRADIENT CONTAINERS VALUES AND THEIR SIGNALS TO SEE IF THE SIGNALS ARE CONSISTENT

    delete[] outputImg;
    delete[] outputGradX;
    delete[] outputGradY;
}
