cmake_minimum_required(VERSION 3.22.1)
project(PM_project)
set(CMAKE_CXX_STANDARD 20)

# Add thread sanitizer flags
set(SANITIZE_THREAD OFF)  # Set this to OFF to disable Thread Sanitizer

if(SANITIZE_THREAD)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread -g -O1")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=thread -g -O1")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=thread")
endif()

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
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} Manager SimTwoInterface Localization OfflineAnalysis AMRController Logger
        UtilsLibrary Visualizer stbImageInit ImGui glfw gl3w)