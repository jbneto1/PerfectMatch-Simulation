#include "Visualizer.h"

Visualizer::Visualizer(Localization &localization, Localization &localization_outliers, std::mutex &PM_m, Logger &logger) : localization(localization),
                                                                                                                            localization_semantics(localization_outliers),
                                                                                                                            newDataAvailable(false),
                                                                                                                            runRenderLoop(true),
                                                                                                                            logger(logger)
{
    windowWidth = 1500;
    windowHeight = 950;
    if (!initialize())
    {
        cleanup();
        throw std::runtime_error("Initialization failed!");
    }

    localUpdate.stepGet = localization.getPM().getStep();
    localUpdate.Qk_covarianceGet = localization.getEKF().getQk();
}

Visualizer::~Visualizer()
{
    this->cleanup();
}

void Visualizer::cleanup()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (window)
    {
        glfwDestroyWindow(window);
        window = nullptr;
    }
    glfwTerminate();
    glDeleteTextures(1, &textureId);
}

bool Visualizer::initialize()
{
    int major, minor, revision;
    glfwGetVersion(&major, &minor, &revision);
    logger.info("GLFW version: " + std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(revision));

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
    {
        logger.error("Failed to initialize GLFW!");
        return false;
    }

    if (!setupGlfwWindow() || !setupGLLoaderAndImGui() || !setupTexture())
    {
        return false;
    }

    checkGlError();
    logger.info("Visualizer initialized.");
    glfwMakeContextCurrent(nullptr);
    return true;
}

bool Visualizer::setupGlfwWindow()
{
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    window = glfwCreateWindow(windowWidth, windowHeight, "Robot Localization", nullptr, nullptr); // 1040 Change to desired size
    glfwShowWindow(window);
    if (!window)
    {
        logger.error("Failed to create GLFW window!");
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    return true;
}

bool Visualizer::setupGLLoaderAndImGui()
{
    if (gl3wInit() != 0)
    {
        logger.error("Failed to initialize OpenGL loader!");
        glfwTerminate();
        return false;
    }
    checkGlError();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    const char *glsl_version = "#version 410 core";
    ImGui_ImplOpenGL3_Init(glsl_version);
    checkGlError();

    return true;
}

bool Visualizer::setupTexture()
{
    int texChannels;
    logger.info("CWD: " + std::filesystem::current_path().string());
    unsigned char *pixels = stbi_load("../srcPython/map/matrix.png", &texWidth, &texHeight, &texChannels,
                                      STBI_rgb_alpha);
    if (!pixels)
    {
        logger.error("Failed to load texture image!");
        logger.error(std::string("STBI Error: ") + stbi_failure_reason());
        return false;
    }

    x_center = texWidth / 2;
    y_center = texHeight / 2;
    x_scale = texWidth / 1.68f;
    y_scale = texHeight / 1.18f;

    glGenTextures(1, &textureId);
    checkGlError();
    glBindTexture(GL_TEXTURE_2D, textureId);
    checkGlError();

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    checkGlError();

    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, texWidth, texHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    checkGlError();

    stbi_image_free(pixels);

    checkGlError();
    return true;
}

void Visualizer::update(const Pose &groundTruth, const Pose &estimatedPose, const std::optional<std::array<LaserPoint, 720>> &laserPoint,
                        const Pose &ePoseOutliers, const std::optional<std::array<LaserPoint, 720>> &laserPointOutliers, const int laserRejectI)
{
    visDataBack.groundTruth = groundTruth;
    visDataBack.estimatedPose = estimatedPose;
    visDataBack.drawLaser = laserPoint.has_value();
    visDataBack.freq_localization = localization.getFreq();

    visDataBack.estimatedPoseOutliers = ePoseOutliers;
    visDataBack.drawLaserOutliers = laserPointOutliers.has_value();

    if (visDataBack.drawLaser)
    {
        visDataBack.laserPoint = laserPoint.value();
    }

    if (visDataBack.drawLaserOutliers)
    {
        visDataBack.laserPointOutliers = laserPointOutliers.value();
    }

    {
        std::lock_guard<std::mutex> lock(cv_m);

        // swap buffers
        std::swap(visDataBack, visDataSwap);

        // local update
        localUpdate.PMError = localization.getPM().getError();
        localUpdate.PMError_semantics = localization_semantics.getPM().getError();

        if (localUpdate.hasStepChanged)
        {
            localization.getPM().setStep(localUpdate.stepSet);
            localization_semantics.getPM().setStep(localUpdate.stepSet);
            localUpdate.stepGet = localization.getPM().getStep();
            localUpdate.hasStepChanged = false; // Reset the flag
        }

        if (localUpdate.hasQkChanged)
        {
            localization.getEKF().setQ(localUpdate.Qk_covarianceSet);
            localization_semantics.getEKF().setQ(localUpdate.Qk_covarianceSet);
            localUpdate.Qk_covarianceGet = localization.getEKF().getQk();
            localUpdate.hasQkChanged = false; // Reset the flag
        }

        if (localUpdate.hasPoseChanged)
        {
            localization.setPose(localUpdate.newPose);
            localization_semantics.setPose(localUpdate.newPose);
            localUpdate.hasPoseChanged = false; // Reset the flag
        }

        if (localUpdate.hasSafetyChanged)
        {
            localization_semantics.getPM().setSafetyThreshold(localUpdate.safetyThresholdSet);
            localUpdate.safetyThresholdGet = localization_semantics.getPM().getSafetyThreshold();
            localUpdate.hasSafetyChanged = false;
        }
    }

    newDataAvailable.store(true);
    cv.notify_one();
}

void Visualizer::handleEvents()
{
    std::unique_lock<std::mutex> lk(cv_m);
    cv.wait(lk, [this]()
            { return newDataAvailable.load(); });
    if (!runRenderLoop.load())
        return; // Early exit if needed
    glfwMakeContextCurrent(window);
}

void Visualizer::setupImGuiFrame()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Create a new ImGui window
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(windowWidth, windowHeight));
    ImGui::Begin("Robot Localization", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoTitleBar);

    ImGui::SetWindowFontScale(2); // Change the scale value to what suits you
    // Set the cursor position
    ImGui::SetCursorPos(ImVec2(0, 0));

    // Draw the map
    ImGui::Image((void *)(intptr_t)textureId, ImVec2(texWidth, texHeight));
}

void Visualizer::drawUIElements()
{

    // Display the pose data
    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();

    float size = 14.0f; // adjust size to match your font
    float halfBase = size / 2.0f;

    // Draw triangles for GroundTruth and EstimatedPose
    drawTriangle(draw_list, visDataFront.groundTruth, ImColor(255, 0, 0)); // red
    checkGlError();
    drawTriangle(draw_list, visDataFront.estimatedPose, ImColor(0, 0, 255)); // blue
    checkGlError();
    drawTriangle(draw_list, visDataFront.estimatedPoseOutliers, ImColor(255, 128, 0));

    draw_list->AddCircle(ImVec2(x_center, y_center), 10, IM_COL32(0, 255, 0, 255), 0, true);

    if (visDataFront.drawLaser)
    {
        drawLidarPoints(draw_list, visDataFront.estimatedPose, visDataFront.laserPoint, IM_COL32(0, 0, 255, 255));
        checkGlError();
    }

    if (visDataFront.drawLaserOutliers)
    {
        drawLidarPoints(draw_list, visDataFront.estimatedPoseOutliers, visDataFront.laserPointOutliers, IM_COL32(255, 128, 0, 255));
        checkGlError();
    }

    draw_list->AddTriangleFilled(
        ImVec2(p.x + halfBase, p.y + 5),               // Top vertex
        ImVec2(p.x, p.y + size + 5),                   // Bottom left vertex
        ImVec2(p.x + size, p.y + size + 5),            // Bottom right vertex
        ImColor(0, 0, 255));                           // Red filled Triangle // Red filled Triangle
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 20); // Push cursor to right by 50 units
    ImGui::Text("EKF Pose [m]:  x=%.3f, y=%.3f, theta=%.3fº", visDataFront.estimatedPose.getX(), visDataFront.estimatedPose.getY(),
                visDataFront.estimatedPose.getThetaDeg());

    p = ImGui::GetCursorScreenPos();

    draw_list->AddTriangleFilled(
        ImVec2(p.x + halfBase, p.y + 5),               // Top vertex
        ImVec2(p.x, p.y + size + 5),                   // Bottom left vertex
        ImVec2(p.x + size, p.y + size + 5),            // Bottom right vertex
        ImColor(255, 0, 0));                           // Red filled Triangle
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 20); // Push cursor to right by 50 units
    ImGui::Text("Ground Truth [m]:  x=%.3f, y=%.3f, theta=%.3fº", visDataFront.groundTruth.getX(), visDataFront.groundTruth.getY(),
                visDataFront.groundTruth.getThetaDeg());

    p = ImGui::GetCursorScreenPos();

    draw_list->AddTriangleFilled(
        ImVec2(p.x + halfBase, p.y + 5),               // Top vertex
        ImVec2(p.x, p.y + size + 5),                   // Bottom left vertex
        ImVec2(p.x + size, p.y + size + 5),            // Bottom right vertex
        ImColor(255, 128, 0));                         // Red filled Triangle
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 20); // Push cursor to right by 50 units
    ImGui::Text("EKF Pose w/ Outliers [m]:  x=%.3f, y=%.3f, theta=%.3fº", visDataFront.estimatedPoseOutliers.getX(), visDataFront.estimatedPoseOutliers.getY(),
                visDataFront.estimatedPoseOutliers.getThetaDeg());

    Pose temp = visDataFront.estimatedPose - visDataFront.groundTruth;

    Pose tempOutliers = visDataFront.estimatedPoseOutliers - visDataFront.groundTruth;

    ImGui::Text("Pose error [m]:  x=%.3f, y=%.3f, theta=%.3fº", temp.getX(),
                temp.getY(), temp.getThetaDeg());

    ImGui::Text("Pose error w/ Outliers [m]:  x=%.3f, y=%.3f, theta=%.3fº", tempOutliers.getX(),
                tempOutliers.getY(), tempOutliers.getThetaDeg());

    ImGui::Text("Localization freq [Hz]: %.2f", 40.0);

    {
        std::lock_guard<std::mutex> lock(cv_m);
        ImGui::Text("PM error [m]: %.3f", localUpdate.PMError);
        ImGui::Text("PM error w/ Outliers [m]: %.3f", localUpdate.PMError_semantics);
    }

    ImGui::SetCursorPosY(0);
    ImGui::SetCursorPosX(texWidth);

    static double k = 0.0f;
    ImGui::PushItemWidth(160);
    if (ImGui::InputDouble("Step scale", &k, 0.0005, 0.0005, "%.4f"))
    {
        if (k < 0)
            k = 0;
    }
    ImGui::PopItemWidth();
    ImGui::SetCursorPosX(texWidth);

    if (ImGui::Button("Set step"))
    {
        std::lock_guard<std::mutex> lock(cv_m);
        localUpdate.hasStepChanged = true;
        localUpdate.stepSet = k;
    }

    ImGui::SameLine();

    std::string stepScaleText = "Current step: ";
    {
        std::lock_guard<std::mutex> lock(cv_m);
        stepScaleText.append(fmt::format("{:.4f}", localUpdate.stepGet));
    }
    ImGui::Text("%s", stepScaleText.c_str());
    ImGui::NewLine();

    ImGui::SetCursorPosX(texWidth);
    static double Qk = 0.0f;
    ImGui::PushItemWidth(180);
    if (ImGui::InputDouble("Process Model Cov", &Qk, 0.01, 0.01, "%.5f"))
    {
        if (Qk < 0)
            Qk = 0;
    }
    ImGui::PopItemWidth();

    ImGui::SetCursorPosX(texWidth);

    if (ImGui::Button("Set Qk_covariance"))
    {
        std::lock_guard<std::mutex> lock(cv_m);
        localUpdate.hasQkChanged = true;
        localUpdate.Qk_covarianceSet = Qk;
    }
    std::string QkText = "Current Qk: ";
    {
        std::lock_guard<std::mutex> lock(cv_m);
        QkText.append(fmt::format("{:.5f}", localUpdate.Qk_covarianceGet));
    }
    ImGui::SetCursorPosX(texWidth);
    ImGui::Text("%s", QkText.c_str());

    ImGui::SetCursorPosX(texWidth);
    std::string RkText = "Rk diag: " + fmt::format("0.001");
    ImGui::Text("%s", RkText.c_str());

    ImGui::NewLine();

    ImGui::SetCursorPosX(texWidth);

    static double x = 0.0f, y = 0.0f, theta_deg = 0.0f;
    static Pose pose;
    ImGui::PushItemWidth(80);
    bool x_changed = ImGui::InputDouble("input X [m]", &x, 0.0, 0.0, "%.3f");
    ImGui::SetCursorPosX(texWidth);
    bool y_changed = ImGui::InputDouble("input Y [m]", &y, 0.0, 0.0, "%.3f");
    ImGui::SetCursorPosX(texWidth);
    bool theta_changed = ImGui::InputDouble("input Theta [deg]", &theta_deg, 0.0, 0.0, "%.3f");
    ImGui::SetCursorPosX(texWidth);
    ImGui::PopItemWidth();
    if (x_changed | y_changed | theta_changed)
    {
        double theta_rad = theta_deg * (M_PI / 180); // Convert from degree to radians
        pose.setX(x);
        pose.setY(y);
        pose.setTheta(theta_rad);
    }

    ImGui::SetCursorPosX(texWidth);

    if (ImGui::Button("Set Pose"))
    {
        std::lock_guard<std::mutex> lock(cv_m);
        localUpdate.hasPoseChanged = true;
        localUpdate.newPose = pose;
    }

    ImGui::SameLine();

    if (ImGui::Button("Reset"))
    {
        std::lock_guard<std::mutex> lock(cv_m);
        localUpdate.hasPoseChanged = true;
        localUpdate.newPose = visDataFront.groundTruth;
    }

    static int safeThresh = 0;

    ImGui::SetCursorPosX(texWidth);
    ImGui::PushItemWidth(160);
    if (ImGui::InputInt("Input Thresh", &safeThresh, 1, 1, 0))
    {
        if (safeThresh < 0)
            safeThresh = 0;
    }
    ImGui::PopItemWidth();

    ImGui::SetCursorPosX(texWidth);

    if (ImGui::Button("Set safety threshold"))
    {
        std::lock_guard<std::mutex> lock(cv_m);
        localUpdate.hasSafetyChanged = true;
        localUpdate.safetyThresholdSet = safeThresh;
    }

    std::string safeThreshStr = "Current safeThresh: ";
    {
        std::lock_guard<std::mutex> lock(cv_m);
        safeThreshStr.append(fmt::format("{}", localUpdate.safetyThresholdGet));
    }
    ImGui::SetCursorPosX(texWidth);
    ImGui::Text("%s", safeThreshStr.c_str());

    checkGlError();
}

void Visualizer::finishRender()
{
    ImGui::End();
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    checkGlError();
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    checkGlError();
    glClear(GL_COLOR_BUFFER_BIT);
    checkGlError();

    // Render the ImGui content
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Swap front and back buffers
    checkGlError();
    glfwSwapBuffers(window);
    checkGlError();
}

void Visualizer::updateDataAvailability()
{
    glfwPollEvents();
    newDataAvailable.store(false);
}

void Visualizer::swapBuffers()
{
    std::lock_guard<std::mutex> lock(cv_m);
    std::swap(visDataSwap, visDataFront);
}

void Visualizer::render()
{
    // Wait for readiness signal

    {
        std::unique_lock<std::mutex> lock(readinessMutex);
        readinessCV.wait(lock, [this]
                         { return isReadyForRendering; });
    }

    glfwMakeContextCurrent(window);

    while (runRenderLoop.load() && (!glfwWindowShouldClose(window)))
    {
        handleEvents();
        swapBuffers();
        setupImGuiFrame();
        drawUIElements();
        finishRender();
        updateDataAvailability();
    }
}

void Visualizer::drawTriangle(ImDrawList *draw_list, const Pose &robot, const ImColor &color) const
{

    // get the pose i3
    double x = robot.getX();
    double y = robot.getY();
    double theta = robot.getTheta();
    // Set fixed triangle size and define isosceles triangle
    const float triangleHeight = 25.0f;                // fixed size
    const float triangleHalfBase = triangleHeight / 3; // adjust for isosceles triangle
    const float centroidOffset = triangleHeight * 2.0f / 3.0f;

    ImVec2 vertices[3];

    // calculate triangle vertices relative to the centroid, isosceles triangle
    vertices[0].x = centroidOffset; // apex pointing right
    vertices[0].y = 0;

    vertices[1].x = -centroidOffset / 2;
    vertices[1].y = -triangleHalfBase;

    vertices[2].x = -centroidOffset / 2;
    vertices[2].y = triangleHalfBase;

    // apply the pose rotation to the vertices
    for (auto &vertice : vertices)
    {
        double temp_x = vertice.x;
        double temp_y = vertice.y;
        vertice.x = temp_x * cos(theta) - temp_y * sin(theta);
        vertice.y = temp_x * sin(theta) + temp_y * cos(theta);
    }

    // translate the vertices to their final position in the map, with flipped y-axis
    for (auto &vertice : vertices)
    {
        vertice.x = x * x_scale + vertice.x + x_center;
        vertice.y = -y * y_scale - vertice.y + y_center; // negate y values
    }

    // draw the triangle
    draw_list->AddTriangleFilled(vertices[0], vertices[1], vertices[2], color);
}

void Visualizer::glfw_error_callback(int error, const char *description)
{
    auto &log = Logger::getInstance(spdlog::level::level_enum::debug);
    log.error("Glfw error: " + std::to_string(error) + ". " + description);
}

#ifdef DEBUG
void Visualizer::checkGlError()
{
    auto &log = Logger::getInstance(spdlog::level::level_enum::debug);
    GLenum err;
    do
    {
        err = glGetError();
        if (err != GL_NO_ERROR)
            log.error("OpenGL error: " + std::to_string(err));
    } while (err != GL_NO_ERROR);
}
#else

void Visualizer::checkGlError() {}

#endif

void Visualizer::drawLidarPoints(ImDrawList *draw_list, const Pose &pose, const std::array<LaserPoint, 720> &laserP,
                                 const ImColor &color) const
{

    // get the pose information
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getTheta();

    // Set fixed triangle size and define isosceles triangle
    const float h = 0.025f;   // triangle height
    const float b = h / 2.0f; // triangle base

    // Define triangle vertices relative to its centroid in robot's frame
    ImVec2 vertices[3];
    vertices[0] = ImVec2(h / 2, 0);       // tip
    vertices[1] = ImVec2(-h / 2, -b / 2); // bottom left
    vertices[2] = ImVec2(-h / 2, b / 2);  // bottom right

    ImU32 col32 = ImGui::ColorConvertFloat4ToU32(color.Value);

    for (size_t i = 0; i < laserP.size(); i += 8)
    {
        const auto &point = laserP[i];

        if ((point.getD() <= 0) || (!point.getIsBeamValid()))
            continue;

        // Transform from robot's frame to global frame
        double global_x = x + point.getX() * cos(theta) - point.getY() * sin(theta);
        double global_y = y + point.getX() * sin(theta) + point.getY() * cos(theta);

        // Transform to matrix frame
        float image_x = global_x * x_scale + x_center;
        float image_y = -global_y * y_scale + y_center;

        draw_list->AddCircleFilled(ImVec2(image_x, image_y), 3.5, color, 0);

        double dx = point.getDx();
        double dy = point.getDy();

        // Normalize the gradient vector
        double magnitude = std::sqrt(dx * dx + dy * dy);
        if (magnitude == 0)
            continue; // Avoid division by zero
        double normalized_grad_x = dx / magnitude;
        double normalized_grad_y = dy / magnitude;

        // Compute the gradient direction
        double angle = atan2(normalized_grad_y, normalized_grad_x);

        // Rotate and scale the triangle vertices to point in the gradient direction
        ImVec2 rotated_vertices[3];
        for (int j = 0; j < 3; j++)
        {
            float temp_x = vertices[j].x;
            float temp_y = vertices[j].y;
            rotated_vertices[j].x = (temp_x * cos(angle) - temp_y * sin(angle)) * x_scale;
            rotated_vertices[j].y = (temp_x * sin(angle) + temp_y * cos(angle)) * y_scale;
        }

        // Translate the triangle vertices to the laser point's position in the image's frame
        for (int j = 0; j < 3; j++)
        {
            rotated_vertices[j].x += image_x;
            rotated_vertices[j].y = image_y - rotated_vertices[j].y; // Invert the y-axis
        }

        // Draw the triangle
        draw_list->AddTriangleFilled(rotated_vertices[0], rotated_vertices[1], rotated_vertices[2],
                                     col32);
    }
}

void Visualizer::stop()
{
    runRenderLoop.store(false);
    newDataAvailable.store(true);
    cv.notify_all();

    if (window)
    {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}