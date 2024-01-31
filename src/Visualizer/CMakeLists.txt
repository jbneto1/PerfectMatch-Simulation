# Visualizer/CMakeLists.txt

# These packages are required by the Visualizer component
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# List of source files for the Visualizer library
set(VISUALIZER_SRC
    Visualizer.cpp
)

# List of source files for ImGui that are used by the Visualizer
set(IMGUI_SRC
    ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui_impl_glfw.cpp
    ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui.cpp
    ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui_impl_opengl3.cpp
    ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui_draw.cpp
    ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui_widgets.cpp
    ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui_tables.cpp
)

# Compile ImGui into a library
add_library(ImGui ${IMGUI_SRC})

# gl3w source files
set(GL_SRC
    ${PROJECT_SOURCE_DIR}/include/third_party/GL/gl3w.c
)

# Compile gl3w into a library
add_library(gl3w ${GL_SRC})

# Create the Visualizer library
add_library(Visualizer ${VISUALIZER_SRC})

# The Visualizer includes headers from the 'include' directory and 'src' directory
target_include_directories(Visualizer
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/src/       
)

add_library(stbImageInit ${PROJECT_SOURCE_DIR}/include/third_party/stb/stb_image_init.cpp)

# Link Visualizer with its dependencies
target_link_libraries(Visualizer
    PRIVATE
        stbImageInit
        ImGui
        glfw
        gl3w
        OpenGL::GL  # Use the imported target provided by find_package
        Localization
        Logger
)