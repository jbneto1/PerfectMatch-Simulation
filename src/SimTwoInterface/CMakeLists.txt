set(SIMTWO_INTERFACE_SRC
        SimTwoInterface.cpp
        )

add_library(SimTwoInterface ${SIMTWO_INTERFACE_SRC})

target_compile_definitions(SimTwoInterface PRIVATE ASIO_STANDALONE)

target_include_directories(SimTwoInterface PUBLIC
        ${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/src/
        ${PROJECT_SOURCE_DIR}/include/third_party/standalone_asio/
        ${PROJECT_SOURCE_DIR}/include/third_party/)

