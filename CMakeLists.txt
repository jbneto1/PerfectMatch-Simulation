cmake_minimum_required(VERSION 3.22.1)
project(PM_project)
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
add_executable(${PROJECT_NAME} src/main.cpp)


# Include the STB headers
target_include_directories(${PROJECT_NAME} PRIVATE
        ${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/src/
        ${PROJECT_SOURCE_DIR}/include/third_party/)

add_library(UtilsLibrary include/utils/utils.cpp)
add_library(stbImageInit include/third_party/stb/stb_image_init.cpp)

# Link OpenGL libraries
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} Manager SimTwoInterface Localization AMRController Logger
        UtilsLibrary Visualizer stbImageInit ImGui glfw gl3w)