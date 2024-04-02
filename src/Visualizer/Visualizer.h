//
// Created by jabra on 7/27/2023.
//

#ifndef PM_PROJECT_VISUALIZER_H
#define PM_PROJECT_VISUALIZER_H

#include <iostream>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <thread>
#include <optional>
#include <filesystem>
#include <opencv4/opencv2/opencv.hpp>

#include "Eigen/Dense"
#include "GL/gl3w.h"
#include "ImGui/imgui.h"
#include "ImGui/imgui_impl_glfw.h"
#include "ImGui/imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include "stb/stb_image.h"

#include "data_structures/data_structures.h"
#include "config/config.h"
#include "Localization/PerfectMatch/PerfectMatch.h"
#include "Logger/logger.h"
#include "Localization/Localization.h"

using Eigen::Matrix3d;

class Visualizer
{
public:
    Visualizer(Localization &localization, Localization &localization_outliers, std::mutex &PM_m, Logger &logger, int safety_thresh);

    ~Visualizer();

    void update(const Pose &groundTruth, const Pose &estimatedPose, const std::optional<std::array<LaserPoint, 720>> &laserPoint,
                const Pose &ePoseOutliers, const std::optional<std::array<LaserPoint, 720>> &laserPointOutliers, const int &laserRejectI, std::vector<BoundingBox> &bboxes);

    void render();

    void stop();

    void cleanup();

    bool isReadyForRendering = false;
    std::mutex readinessMutex;
    std::condition_variable readinessCV;

private:
    // Setup for drawing rectangles

    Logger &logger;
    float x_scale;
    float y_scale;
    float x_center;
    float y_center;

    //
    double windowWidth;
    double windowHeight;

    Localization &localization;
    Localization &localization_semantics;

    std::condition_variable cv;
    std::mutex cv_m;
    std::atomic<bool> newDataAvailable;
    GLFWwindow *window{};
    GLuint textureId{};          // Texture identifier for the map image
    int texWidth{}, texHeight{}; // Texture size
    std::atomic<bool> runRenderLoop;
    VisualizationData visDataFront, visDataBack, visDataSwap;
    LocalizationUpdateData localUpdate;

    Matrix3d camK;
    int safety_threshold;
    cv::Mat image;

    bool initialize();

    void handleEvents();
    void swapBuffers();
    void setupImGuiFrame();
    void drawUIElements();
    void drawCamVis();
    void finishRender();
    void updateDataAvailability();

    void error_callback(int error, const char *description);

    bool setupTexture();

    static void glfw_error_callback(int error, const char *description);

    static void checkGlError();

    void drawTriangle(ImDrawList *draw_list, const Pose &pose, const ImColor &color) const;

    void drawLidarPoints(ImDrawList *draw_list, const Pose &robot, const std::array<LaserPoint, 720> &laserPoint,
                         const ImColor &color) const;

    bool setupGlfwWindow();

    bool setupGLLoaderAndImGui();

    // RObot Camera window
    void DrawLidarPointWithAnnotation(const Vector2d &pImgPx, u_int index, int annotateEveryN, bool isInsideBoundingBox);
    void DrawBoundingBox(BoundingBox &box);
    void DrawCenterAndCorners();
};

#endif // PM_PROJECT_VISUALIZER_H
