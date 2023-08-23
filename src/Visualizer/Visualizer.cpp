#include "Visualizer.h"

Visualizer::Visualizer(PerfectMatch &perfectMatch) : pm(perfectMatch) {
    initialize();
}

Visualizer::~Visualizer() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    glDeleteTextures(1, &textureId);
}

void Visualizer::initialize() {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return;
    setupGlfwWindow();
    setupGLoaderAndImGui();
    setupTexture();
}

void Visualizer::setupGlfwWindow() {
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    window = glfwCreateWindow(800, 920, "Robot Localization", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
}

void Visualizer::setupGLoaderAndImGui() {
    if (gl3wInit() != 0) {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    const char *glsl_version = "#version 420";
    ImGui_ImplOpenGL3_Init(glsl_version);
}

void Visualizer::setupTexture() {
    // Load the texture (image for the robot map)
    int texChannels;
    unsigned char *pixels = stbi_load("../src/Localization/PerfectMatch/Map/RAFmap.png", &texWidth, &texHeight,
                                      &texChannels, STBI_rgb_alpha);
    if (!pixels) {
        std::cerr << "Failed to load texture image!" << std::endl;
        std::cerr << "STBI Error: " << stbi_failure_reason() << std::endl;
        return;
    }

    // After loading the image, modify its pixels
    for (int i = 0; i < texWidth * texHeight * 4; i += 4) {
        // If the pixel is not black
        if (!(pixels[i] == 0 && pixels[i + 1] == 0 && pixels[i + 2] == 0)) {
            // Change it to white but keep the alpha value
            pixels[i] = pixels[i + 1] = pixels[i + 2] = 255;
            pixels[i + 3] = 255;
        }
    }

    ImGui::StyleColorsClassic();

// Generate and bind the texture
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);

// Set the texture's filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    checkGlError();

// Upload the image data to the texture
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, texWidth, texHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    checkGlError();

// Free the image from memory after uploading its data to the texture
    stbi_image_free(pixels);
}


void Visualizer::update(const Pose &groundTruth, const Pose &estimatedPose, const std::array<LaserPoint, 720> &laserPoint) {
    {
        // Use a lock_guard to ensure thread safety when updating the poses
        std::lock_guard<std::mutex> lock(cv_m);
        this->groundTruth = groundTruth;
        this->estimatedPose = estimatedPose;
        this->laserPoint = laserPoint;
        newDataAvailable = true;
    }
    // Notify one waiting thread, if there is one
    cv.notify_one();
}

void Visualizer::render() {
    // Render loop
    while (!glfwWindowShouldClose(window)) {
        // Wait for new data
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait(lk, [this]() { return newDataAvailable; });
        glfwMakeContextCurrent(window);

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Create a new ImGui window
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(800, 920));
        ImGui::Begin("Robot Localization", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoTitleBar);

        ImGui::SetWindowFontScale(2); // Change the scale value to what suits you

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

        p = ImGui::GetCursorScreenPos();

        ImGui::Text("PM error:  %.3f", pm.getError());

        Pose temp = estimatedPose - groundTruth;

        ImGui::Text("Error:  x=%.3f, y=%.3f, theta=%.3fº", temp.getX(),
                    temp.getY(), temp.getThetaDeg());

        // Setup for drawing rectangles
        float x_scale = 295.0f / 0.62f;
        float y_scale = x_scale;
        float x_center = 800.0f / 2;
        float y_center = 566.0f / 2;

        // Draw triangles for GroundTruth and EstimatedPose
        drawTriangle(draw_list, groundTruth, x_scale, y_scale, x_center, y_center, ImColor(255, 0, 0)); // green
        drawTriangle(draw_list, estimatedPose, x_scale, y_scale, x_center, y_center, ImColor(0, 0, 255)); // red
        draw_list->AddCircle(ImVec2(x_center, y_center), 10, IM_COL32(0, 255, 0, 255), 0, true);
        drawLidarPoints(draw_list, groundTruth, laserPoint, x_scale, y_scale, x_center, y_center, IM_COL32(128, 0, 198, 255));

        static float x = 0.0f, y = 0.0f, theta_deg = 0.0f;
        static Pose pose;
        bool x_changed = ImGui::InputFloat("iX", &x);
        bool y_changed = ImGui::InputFloat("iY", &y);
        bool theta_changed = ImGui::InputFloat("iTheta (deg)", &theta_deg);
        if(x_changed | y_changed | theta_changed) {
            float theta_rad = theta_deg * (M_PI / 180);  // Convert from degree to radians
            pose.setX(x);
            pose.setY(y);
            pose.setTheta(theta_rad);
        }

        if (ImGui::Button("Set Pose")) {
            pm.setPose(pose);
            std::cout << pose.getX() << pose.getY() << pose.getThetaDeg() << std::endl;
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
    }
    glfwTerminate();
    exit(EXIT_SUCCESS);
}

void Visualizer::framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    float aspectRatio = (float) width / (float) height;
    glViewport(0, 0, width, height);

    if (width >= height) {
        texWidth = texWidth * aspectRatio;
    } else {
        texHeight = texHeight / aspectRatio;
    }
}

void Visualizer::drawTriangle(ImDrawList *draw_list, const Pose &pose, float x_scale, float y_scale, float x_center,
                              float y_center, const ImColor &color) {

    if (draw_list == nullptr) {
        throw std::invalid_argument("draw_list cannot be nullptr");
    }

    // get the pose information
    float x = pose.getX();
    float y = pose.getY();
    float theta = pose.getTheta();

    // Set fixed triangle size and define isosceles triangle
    float triangleHeight = 25.0f; // fixed size
    float triangleHalfBase = triangleHeight / 3; // adjust for isosceles triangle

    ImVec2 vertices[3];

    // calculate triangle vertices relative to the origin, isosceles triangle
    vertices[0].x = triangleHeight * 2 / 3; // apex pointing right
    vertices[0].y = 0;

    vertices[1].x = -triangleHeight / 3;
    vertices[1].y = triangleHalfBase;

    vertices[2].x = -triangleHeight / 3;
    vertices[2].y = -triangleHalfBase;

    // apply the pose rotation to the vertices
    for (int i = 0; i < 3; i++) {
        float temp_x = vertices[i].x;
        float temp_y = vertices[i].y;
        vertices[i].x = temp_x * cosf(theta) - temp_y * sinf(theta); // negate theta to correct direction
        vertices[i].y = temp_x * sinf(theta) + temp_y * cosf(theta); // negate theta to correct direction
    }

    // translate the vertices to their final position in the map, with flipped y-axis
    for (int i = 0; i < 3; i++) {
        vertices[i].x = x * x_scale + vertices[i].x + x_center;
        vertices[i].y = - y * y_scale - vertices[i].y + y_center; //negate y values
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

void Visualizer::drawLidarPoints(ImDrawList *draw_list, const Pose& robot, const std::array<LaserPoint, 720> &laserPoint,
                                 float x_scale, float y_scale, float x_center, float y_center, const ImColor &color) {

    if (draw_list == nullptr) {
        throw std::invalid_argument("draw_list cannot be nullptr");
    }

    double robot_x = robot.getX();
    double robot_y = robot.getY();
    double robot_theta = robot.getTheta();  // assuming it's in radians

    const float norm_length = 0.02f; // Choose an appropriate length for the gradient arrows

    // Additional parameters for the triangle
    const float half_base_width = 2; // half width of triangle base in pixels

    for (const auto &point: laserPoint) {

        if(point.getD() <= 0) continue;

        if(visualizeRaw) {
            // Plot the raw points without any transformations
            float raw_x = point.getX() * x_scale + x_center;
            float raw_y = -point.getY() * y_scale + y_center;
            draw_list->AddCircleFilled(ImVec2(raw_x, raw_y), 3.5, color, 0);

            continue;  // Skip the rest of the loop for raw visualization
        }

        // Transform from robot's frame to global frame
        double global_x = robot_x + point.getX() * cos(robot_theta) - point.getY() * sin(robot_theta);
        double global_y = robot_y + point.getX() * sin(robot_theta) + point.getY() * cos(robot_theta);

        // Transform from global frame to image frame
        float image_x = global_x * x_scale + x_center;
        float image_y = - global_y * y_scale + y_center; //negate y values for flipping the image vertically

        draw_list->AddCircleFilled(ImVec2(image_x, image_y), 3.5, color, 0);

        double dx = point.getDx();
        double dy = point.getDy();

        // Calculate the direction of the gradient vector
        double magnitude = sqrt(dx * dx + dy * dy);
        double dX_world = dx / magnitude;
        double dY_world = dy / magnitude;

        // Get the triangle tip coordinates
        float triangle_tip_x = image_x + dX_world * norm_length * x_scale;
        float triangle_tip_y = image_y - dY_world * norm_length * y_scale; // negating dY to flip the image vertically

        // calculate the base vertices of the triangle
        float base_vertex1_x = image_x + half_base_width * (-dY_world);
        float base_vertex1_y = image_y - half_base_width * dX_world; // negating dY to flip the image vertically

        float base_vertex2_x = image_x - half_base_width * (-dY_world);
        float base_vertex2_y = image_y + half_base_width * dX_world; // negating dY to flip the image vertically

        ImVec2 triangle_tip(triangle_tip_x, triangle_tip_y);
        ImVec2 base_vertex1(base_vertex1_x, base_vertex1_y);
        ImVec2 base_vertex2(base_vertex2_x, base_vertex2_y);

        draw_list->AddTriangleFilled(triangle_tip, base_vertex1, base_vertex2, IM_COL32(255, 0, 0, 255));
    }
}
