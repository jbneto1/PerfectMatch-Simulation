set(OFFLINEANALYSIS_SRC
        OfflineAnalysis.cpp
        )

add_library(OfflineAnalysis ${OFFLINEANALYSIS_SRC})

target_include_directories(OfflineAnalysis PRIVATE
                        ${PROJECT_SOURCE_DIR}/src/
                        ${PROJECT_SOURCE_DIR}/include/
                        ${PROJECT_SOURCE_DIR}/include/third_party/
                        ${PROJECT_SOURCE_DIR}/include/third_party/eigen-3.4.0/)