//
// Created by jabra on 7/27/2023.
//

#ifndef PM_PROJECT_VISUALIZER_H
#define PM_PROJECT_VISUALIZER_H

#include <GL/gl3w.h>
#include <third_party/ImGui/imgui.h>
#include <third_party/ImGui/imgui_impl_glfw.h>
#include <third_party/ImGui/imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include <third_party/eigen-3.4.0/Eigen/Dense>
#include <config/config.h>
#include <iostream>
#include <condition_variable>
#include <mutex>  // Added for std::mutex
#include <third_party/stb/stb_image.h>
#include "../src/Localization/PerfectMatch/PerfectMatch.h"
#include <Logger/logger.h>
#include <chrono>
#include <thread>
#include "../src/Localization/Localization.h"


class Visualizer {
public:
    explicit Visualizer(Localization &localization);

    ~Visualizer();

    void update(const Pose &groundTruth, const Pose &estimatedPose, const std::array<LaserPoint, 720> &laserPoint);

    void render();

private:

    // Setup for drawing rectangles
    float x_scale;
    float y_scale;
    float x_center;
    float y_center;
    float freq_localization;

    Localization &localization;

    std::condition_variable cv;
    std::mutex cv_m;
    bool newDataAvailable = false;
    Pose groundTruth;
    Pose estimatedPose;
    std::array<LaserPoint, 720> laserPoint;
    GLFWwindow *window{};
    GLuint textureId{};  // Texture identifier for the map image
    int texWidth{}, texHeight{};  // Texture size variables

    bool initialize();

    void cleanup();

    bool setupTexture();

    static void glfw_error_callback(int error, const char *description);

    static void checkGlError();

    void drawTriangle(ImDrawList *draw_list, const Pose &pose, const ImColor &color) const;

    void drawLidarPoints(ImDrawList *draw_list, const Pose &robot, const std::array<LaserPoint, 720> &laserPoint,
                         const ImColor &color) const;

    bool setupGlfwWindow();

    bool setupGLLoaderAndImGui();
};

#endif //PM_PROJECT_VISUALIZER_H
