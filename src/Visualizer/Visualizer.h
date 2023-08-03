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
#include <third_party/eigen-3.4.0//Eigen/Dense>
#include <config/config.h>
#include <iostream>
#include <condition_variable>
#include <third_party/stb/stb_image.h>
#include <../src/Localization/PerfectMatch/PerfectMatch.h>
#include <Logger/logger.h>

class Visualizer {
public:
    Visualizer(PerfectMatch &perfectMatch);

    ~Visualizer();

    void update(const Pose &groundTruth, const Pose &estimatedPose, const std::array<LaserPoint, 720> &laserPoint);

    void render();

private:
    PerfectMatch &pm;

    std::condition_variable cv;
    std::mutex cv_m;
    bool newDataAvailable = false;
    Pose groundTruth;
    Pose estimatedPose;
    std::array<LaserPoint, 720> laserPoint;
    GLFWwindow *window;
    GLuint textureId;  // Added: Texture identifier for the map image
    int texWidth, texHeight;  // Added: Texture size variables
    void initialize();

    static void glfw_error_callback(int error, const char *description);

    void checkGlError();


    void
    drawTriangle(ImDrawList *draw_list, const Pose &pose, float x_scale, float y_scale, float x_center, float y_center,
                 const ImColor &color);

    void drawLidarPoints(ImDrawList *draw_list, const Pose &robot, const std::array<LaserPoint, 720> &laserPoint,
                         float x_scale, float y_scale, float x_center, float y_center, const ImColor &color);

    void setupGlfwWindow();

    void setupGLoaderAndImGui();

    void setupTexture();

    void framebuffer_size_callback(GLFWwindow *window, int width, int height);

    static void framebuffer_size_callback_wrapper(GLFWwindow *window, int width, int height) {
        Visualizer *self = static_cast<Visualizer *>(glfwGetWindowUserPointer(window));
        self->framebuffer_size_callback(window, width, height);
    }
};

#endif //PM_PROJECT_VISUALIZER_H
