cmake_minimum_required(VERSION 3.22.1)
project(logParser)
set(CMAKE_CXX_STANDARD 20)

# The main project directory is three levels up from the current directory of logParser
set(MAIN_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(PROJECT_SOURCE_DIR ${MAIN_PROJECT_DIR})

# Add the subdirectories for Localization and OfflineAnalysis with specified binary directories
add_subdirectory(${MAIN_PROJECT_DIR}/src/Localization ${CMAKE_CURRENT_BINARY_DIR}/Localization)
add_subdirectory(${MAIN_PROJECT_DIR}/src/OfflineAnalysis ${CMAKE_CURRENT_BINARY_DIR}/OfflineAnalysis)
add_subdirectory(${MAIN_PROJECT_DIR}/src/Logger ${CMAKE_CURRENT_BINARY_DIR}/Logger)
add_subdirectory(${MAIN_PROJECT_DIR}/include/utils ${CMAKE_CURRENT_BINARY_DIR}/utils)

# Create the main executable
add_executable(${PROJECT_NAME} logParser.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
                            ${MAIN_PROJECT_DIR}/src
                            ${MAIN_PROJECT_DIR}/include
                            ${MAIN_PROJECT_DIR}/include/third_party/eigen-3.4.0/
                            )  # This is the adjusted correct path

# Link the required libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Logger
        OfflineAnalysis
        Localization
        UtilsLibrary
)
