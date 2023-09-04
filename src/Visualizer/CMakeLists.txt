set(VISUALIZER_SRC
        Visualizer.cpp
        )

# Find OpenGL
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(IMGUI_SRC
        ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui_impl_glfw.cpp
        ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui.cpp
        ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui_impl_opengl3.cpp
        ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui_draw.cpp
        ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui_widgets.cpp
        ${PROJECT_SOURCE_DIR}/include/third_party/ImGui/imgui_tables.cpp)

add_library(ImGui ${IMGUI_SRC})

set(GL_SRC
        ${PROJECT_SOURCE_DIR}/include/GL/gl3w.c)

add_library(gl3w ${GL_SRC})

add_library(Visualizer ${VISUALIZER_SRC})

target_include_directories(Visualizer PRIVATE
                        ${PROJECT_SOURCE_DIR}/include/)

target_link_libraries(Visualizer ImGui glfw gl3w Localization Logger)