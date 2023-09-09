set(LOCALIZATION_SRC
        Localization.cpp
        PerfectMatch/PerfectMatch.cpp
        ExtendedKalmanFilter/ExtendedKalmanFilter.cpp
        PerfectMatch/Map/Map.cpp)

add_library(Localization ${LOCALIZATION_SRC})

target_include_directories(Localization PUBLIC
                        ${PROJECT_SOURCE_DIR}/include/
                        ${PROJECT_SOURCE_DIR}/src/
                        ${PROJECT_SOURCE_DIR}/include/third_party/
                        ${PROJECT_SOURCE_DIR}/include/third_party/eigen-3.4.0/)

target_link_libraries(Localization PUBLIC Logger)