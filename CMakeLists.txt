cmake_minimum_required(VERSION 3.25)
project(AMR_Project)
set(CMAKE_CXX_STANDARD 20)

include_directories(
        ${PROJECT_SOURCE_DIR}/include/config
        ${PROJECT_SOURCE_DIR}/include/third_party
        ${PROJECT_SOURCE_DIR}/include/third_party/standalone_asio
        ${PROJECT_SOURCE_DIR}/include/third_party/eigen-3.4.0
        ${PROJECT_SOURCE_DIR}/src
)

add_definitions(-DASIO_STANDALONE)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} Manager SimTwoInterface Localization AMRController Logger ws2_32)
