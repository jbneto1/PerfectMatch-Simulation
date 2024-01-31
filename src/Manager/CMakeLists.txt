# Manager/CMakeLists.txt

set(MANAGER_SRC
    Manager.cpp
)

add_library(Manager ${MANAGER_SRC})


# Include directories for the Manager component. It's assumed that other components' headers
# are located in the 'src' directory or that their targets export their include directories.
target_include_directories(Manager
PUBLIC
${PROJECT_SOURCE_DIR}/include/third_party/standalone_asio/
# You may not need to include the 'src' directory if each component
# properly exports its own include directories when built.
${PROJECT_SOURCE_DIR}/src/
)

add_definitions(-DASIO_STANDALONE)

target_compile_definitions(SimTwoInterface PRIVATE ASIO_STANDALONE)

# Link the Manager with its dependencies
target_link_libraries(Manager
    PRIVATE
        SimTwoInterface  # Used in Manager implementation
        Localization     # Used in Manager implementation
        Logger           # Used in Manager implementation
        AMRController    # Used in Manager implementation
        Visualizer       # Used in Manager implementation
        OfflineAnalysis  # Used in Manager implementation
)

# If ASIO is not set up as a CMake target, you may need to add it as a system include
# directory to avoid warnings with newer CMake versions. This is done as follows:
# target_include_directories(Manager SYSTEM PRIVATE ${ASIO_INCLUDE_DIR})

# Since we're now using target_link_libraries, we can remove the redundant include
# directories for ASIO and Eigen from target_include_directories.
