# SimTwoInterface/CMakeLists.txt

set(SIMTWO_INTERFACE_SRC
        SimTwoInterface.cpp
)

add_library(SimTwoInterface ${SIMTWO_INTERFACE_SRC})

# Set the ASIO_STANDALONE definition to indicate using standalone ASIO instead of Boost.Asio
target_compile_definitions(SimTwoInterface PRIVATE ASIO_STANDALONE)

# Include directories for SimTwoInterface, these should be the ones needed for the header files used
target_include_directories(SimTwoInterface
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/include/third_party/
        ${PROJECT_SOURCE_DIR}/include/third_party/standalone_asio/
        ${PROJECT_SOURCE_DIR}/src/
)


# Link the SimTwoInterface with its dependencies
target_link_libraries(SimTwoInterface
    PRIVATE
        Logger  # Logger is used in the implementation, so we link it privately
        Localization  # Localization is used in the implementation, so we link it privately
        AMRController  # AMRController is used in the implementation, so we link it privately
        # UtilsLibrary  # If utils are used in SimTwoInterface, link it here
)
