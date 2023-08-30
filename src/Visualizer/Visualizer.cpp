#include "Visualizer.h"

// Named constants for better clarity
const float TRIANGLE_HEIGHT = 25.0f;
const float TRIANGLE_HALF_BASE = TRIANGLE_HEIGHT / 3.0f;
const float CENTROID_OFFSET = TRIANGLE_HEIGHT * 2.0f / 3.0f;

Visualizer::Visualizer(PerfectMatch &perfectMatch) : pm(perfectMatch){
    if (!initialize()) {
        std::cerr << "Initialization failed!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

Visualizer::~Visualizer() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    glDeleteTextures(1, &textureId);
}

bool Visualizer::initialize() {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW!" << std::endl;
        return false;
    }

    if (!setupGlfwWindow() || !setupGLLoaderAndImGui() || !setupTexture()) {
        return false;
    }

    return true;
}

bool Visualizer::setupGlfwWindow() {
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    window = glfwCreateWindow(996, 1000, "Robot Localization", nullptr, nullptr); // Change to desired size
    if (window == nullptr) {
        std::cerr << "Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    return true;
}

bool Visualizer::setupGLLoaderAndImGui() {
    if (gl3wInit() != 0) {
        std::cerr << "Failed to initialize OpenGL loader!" << std::endl;
        glfwTerminate();
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    const char *glsl_version = "#version 420";
    ImGui_ImplOpenGL3_Init(glsl_version);

    return true;
}

bool Visualizer::setupTexture() {
    int texChannels;
    unsigned char *pixels = stbi_load("../srcPython/map/matrix.png", &texWidth, &texHeight, &texChannels,
                                      STBI_rgb_alpha);
    if (!pixels) {
        std::cerr << "Failed to load texture image!" << std::endl;
        std::cerr << "STBI Error: " << stbi_failure_reason() << std::endl;
        return false;
    }

    x_center = texWidth / 2;
    y_center = texHeight / 2;
    x_scale = texWidth / 1.68f;
    y_scale = texHeight / 1.18f;

    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    checkGlError();

    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, texWidth, texHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    checkGlError();

    stbi_image_free(pixels);

    return true;
}

void
Visualizer::update(const Pose &groundTruth, const Pose &estimatedPose, const std::array<LaserPoint, 720> &laserPoint) {
    std::lock_guard <std::mutex> lock(cv_m);
    this->groundTruth = groundTruth;
    this->estimatedPose = estimatedPose;
    this->laserPoint = laserPoint;
    this->freq_PM = pm.getFreq();
    newDataAvailable = true;

    cv.notify_one();
}

void Visualizer::render() {
    // Render loop
    while (!glfwWindowShouldClose(window)) {
        // Wait for new data
        std::unique_lock <std::mutex> lk(cv_m);
        cv.wait(lk, [this]() { return newDataAvailable; });
        glfwMakeContextCurrent(window);

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Create a new ImGui window
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(texWidth, texHeight + 310));
        ImGui::Begin("Robot Localization", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoTitleBar);

        ImGui::SetWindowFontScale(2); // Change the scale value to what suits you
// Set the cursor position
        ImGui::SetCursorPos(ImVec2(0, 0));

// Draw the map
        ImGui::Image((void *) (intptr_t) textureId, ImVec2(texWidth, texHeight));
        // Display the pose data
        ImDrawList *draw_list = ImGui::GetWindowDrawList();
        ImVec2 p = ImGui::GetCursorScreenPos();


        float size = 14.0f; //adjust size to match your font
        float halfBase = size / 2.0f;

        draw_list->AddTriangleFilled(
                ImVec2(p.x + halfBase, p.y + 5),               // Top vertex
                ImVec2(p.x, p.y + size + 5),                   // Bottom left vertex
                ImVec2(p.x + size, p.y + size + 5),            // Bottom right vertex
                ImColor(0, 0, 255)
        );  // Red filled Triangle // Red filled Triangle
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 20); // Push cursor to right by 50 units
        ImGui::Text("Estimated Pose:  x=%.3f, y=%.3f, theta=%.3fº", estimatedPose.getX(), estimatedPose.getY(),
                    estimatedPose.getThetaDeg());

        p = ImGui::GetCursorScreenPos();

        draw_list->AddTriangleFilled(
                ImVec2(p.x + halfBase, p.y + 5),               // Top vertex
                ImVec2(p.x, p.y + size + 5),                   // Bottom left vertex
                ImVec2(p.x + size, p.y + size + 5),            // Bottom right vertex
                ImColor(255, 0, 0)
        );  // Red filled Triangle
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 20); // Push cursor to right by 50 units
        ImGui::Text("Ground Truth:  x=%.3f, y=%.3f, theta=%.3fº", groundTruth.getX(), groundTruth.getY(),
                    groundTruth.getThetaDeg());

        ImGui::Text("PM error: %.3f", pm.getError());

        ImGui::Text("PM freq: %.2f Hz", freq_PM);

        Pose temp = estimatedPose - groundTruth;

        ImGui::Text("Error:  x=%.3f, y=%.3f, theta=%.3fº", temp.getX(),
                    temp.getY(), temp.getThetaDeg());

        // Draw triangles for GroundTruth and EstimatedPose
        drawTriangle(draw_list, groundTruth, ImColor(255, 0, 0)); // green
        drawTriangle(draw_list, estimatedPose, ImColor(0, 0, 255)); // blue
        draw_list->AddCircle(ImVec2(x_center, y_center), 10, IM_COL32(0, 255, 0, 255), 0, true);
        drawLidarPoints(draw_list, estimatedPose, laserPoint, IM_COL32(128, 0, 198, 255));

        static double x = 0.0f, y = 0.0f, theta_deg = 0.0f;
        static Pose pose;
        bool x_changed = ImGui::InputDouble("iX", &x);
        bool y_changed = ImGui::InputDouble("iY", &y);
        bool theta_changed = ImGui::InputDouble("iTheta (deg)", &theta_deg);
        if (x_changed | y_changed | theta_changed) {
            double theta_rad = theta_deg * (M_PI / 180);  // Convert from degree to radians
            pose.setX(x);
            pose.setY(y);
            pose.setTheta(theta_rad);
        }

        if (ImGui::Button("Set Pose")) {
            pm.setPose(pose);
        }

        ImGui::SameLine();

        if (ImGui::Button("Reset")) {
            pm.setPose(groundTruth);
        }

        // Finish the ImGui window
        ImGui::End();

        // Render ImGui
        ImGui::Render();

        // Set the viewport and clear the screen
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Render the ImGui content
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
        newDataAvailable = false;

        //TODO TALK TO PACO ABOUT INCONSISTENT SCALING, dont understand dtheta, show the way the maps were computed , orientatio and positions where it diverges, NON-SQUARE PX, GRADIENT ORIENTATION,
        // APP BURNING MY CPU EVEN WITH SLEEP
}
    glfwTerminate();
    exit(EXIT_SUCCESS);
}

void Visualizer::drawTriangle(ImDrawList *draw_list, const Pose &robot, const ImColor &color) const {

    // get the pose information
    double x = robot.getX();
    double y = robot.getY();
    double theta = robot.getTheta();
// Set fixed triangle size and define isosceles triangle
    const float triangleHeight = 25.0f; // fixed size
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
    for (auto &vertice: vertices) {
        double temp_x = vertice.x;
        double temp_y = vertice.y;
        vertice.x = temp_x * cos(theta) - temp_y * sin(theta);
        vertice.y = temp_x * sin(theta) + temp_y * cos(theta);
    }

    // translate the vertices to their final position in the map, with flipped y-axis
    for (auto &vertice: vertices) {
        vertice.x = x * x_scale + vertice.x + x_center;
        vertice.y = -y * y_scale - vertice.y + y_center; //negate y values
    }

    // draw the triangle
    draw_list->AddTriangleFilled(vertices[0], vertices[1], vertices[2], color);
}


void Visualizer::glfw_error_callback(int error, const char *description) {
    std::cerr << "Glfw Error " << error << ": " << description << std::endl;
}

void Visualizer::checkGlError() {
    GLenum err;
    do {
        err = glGetError();
        if (err != GL_NO_ERROR)
            std::cerr << "OpenGL error: " << err << std::endl;
    } while (err != GL_NO_ERROR);
}

void Visualizer::drawLidarPoints(ImDrawList *draw_list, const Pose &pose, const std::array<LaserPoint, 720> &laserP,
                                 const ImColor &color) const {

    // get the pose information
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getTheta();
// Set fixed triangle size and define isosceles triangle
    const float triangleHeight = 0.02f; // fixed size
    const float triangleHalfBase = triangleHeight / 3; // adjust for isosceles triangle
    const float centroidOffset = triangleHeight * 2.0f / 3.0f;

    ImVec2 original_vertices[3];
    original_vertices[0].x = centroidOffset;
    original_vertices[0].y = 0;
    original_vertices[1].x = -centroidOffset / 2;
    original_vertices[1].y = -triangleHalfBase;
    original_vertices[2].x = -centroidOffset / 2;
    original_vertices[2].y = triangleHalfBase;


    for (const auto &point: laserP) {

        if (point.getD() <= 0) continue;

        // Transform from robot's frame to global frame
        double global_x = x + point.getX() * cos(theta) - point.getY() * sin(theta);
        double global_y = y + point.getX() * sin(theta) + point.getY() * cos(theta);

        // Transform to matrix frame
        float image_x = global_x * x_scale + x_center;
        float image_y = -global_y * y_scale + y_center;

        draw_list->AddCircleFilled(ImVec2(image_x, image_y), 3.5, color, 0);

        double dx = point.getDx();
        double dy = point.getDy();

        // Make the gradient vectors as uniform
        double magnitude = std::sqrt(dx * dx + dy * dy);
        if (magnitude == 0) continue;  // Avoid division by zero
        double normalized_grad_x_world = dx / magnitude;
        double normalized_grad_y_world = dy / magnitude;

        double angle = atan2(normalized_grad_y_world, normalized_grad_x_world);

        angle = normalizeAngle(angle);

        // apply the pose rotation to the vertices
        // Use a temporary array for the rotated vertices
        ImVec2 rotated_vertices[3];
        for (int i = 0; i < 3; i++) {
            double temp_x = original_vertices[i].x;
            double temp_y = original_vertices[i].y;
            rotated_vertices[i].x = temp_x * cos(angle) - temp_y * sin(angle);
            rotated_vertices[i].y = temp_x * sin(angle) + temp_y * cos(angle);
        }


        float triangle_tip_x = image_x + rotated_vertices[0].x * x_scale;
        float triangle_tip_y = image_y - rotated_vertices[0].y * y_scale;  // Note the negative sign for y-axis inversion

        float base_vertex1_x = image_x + rotated_vertices[1].x * x_scale;
        float base_vertex1_y = image_y - rotated_vertices[1].y * y_scale;  // Note the negative sign for y-axis inversion

        float base_vertex2_x = image_x + rotated_vertices[2].x * x_scale;
        float base_vertex2_y = image_y - rotated_vertices[2].y * y_scale;  // Note the negative sign for y-axis inversion


        ImVec2 triangle_tip(triangle_tip_x, triangle_tip_y);
        ImVec2 base_vertex1(base_vertex1_x, base_vertex1_y);
        ImVec2 base_vertex2(base_vertex2_x, base_vertex2_y);

        draw_list->AddTriangleFilled(triangle_tip, base_vertex1, base_vertex2, IM_COL32(255, 0, 0, 255));
    }
}