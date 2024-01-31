set(LOGGER_SRC
        logger.cpp
        )

add_library(Logger ${LOGGER_SRC})

target_include_directories(Logger PUBLIC
                        ${PROJECT_SOURCE_DIR}/include/third_party/
                        ${PROJECT_SOURCE_DIR}/include/third_party/eigen-3.4.0/
                        ${PROJECT_SOURCE_DIR}/include/)