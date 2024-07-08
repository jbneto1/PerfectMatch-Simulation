# OfflineAnalysis/CMakeLists.txt

set(OFFLINEANALYSIS_SRC
        OfflineAnalysis.cpp
)

add_library(OfflineAnalysis ${OFFLINEANALYSIS_SRC})

# Include directories for OfflineAnalysis
# Eigen is included because the OfflineAnalysis component uses Eigen in its implementation.
# However, if Eigen is used in the headers that are included by other targets,
# it should be PUBLIC instead of PRIVATE.
target_include_directories(OfflineAnalysis
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/src/
        ${OpenCV_INCLUDE_DIRS}
)

# Link the OfflineAnalysis with its dependencies
target_link_libraries(OfflineAnalysis
    PRIVATE
        Localization  # Localization is used in the implementation, so we link it privately
        Logger        # Logger is used in the implementation, so we link it privately
        ${OpenCV_LIBS}
)
