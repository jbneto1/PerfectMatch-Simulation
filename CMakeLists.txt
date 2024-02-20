cmake_minimum_required(VERSION 3.22.1)
project(PM_project)
set(CMAKE_CXX_STANDARD 20)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Add thread sanitizer flags if needed
set(SANITIZE_THREAD OFF)  # Set this to OFF to disable Thread Sanitizer
if(SANITIZE_THREAD)
    add_compile_options(-fsanitize=thread -g -O1)
    link_libraries(-fsanitize=thread)
endif()

# Add the subdirectories that contain the CMakeLists.txt for each component
add_subdirectory(${PROJECT_SOURCE_DIR}/include/utils)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/SimTwoInterface)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/Localization)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/AMRController)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/Logger)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/Manager)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/Visualizer)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/OfflineAnalysis)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests)

# Create the main executable
add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
                            ${PROJECT_SOURCE_DIR}/src
                            ${PROJECT_SOURCE_DIR}/include/third_party/eigen-3.4.0/
                            ${OpenCV_INCLUDE_DIRS})

# Since the main executable depends on the Manager, Logger, and config,
# we need to link those. Other components will be linked if used directly.
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Manager
        Logger
)