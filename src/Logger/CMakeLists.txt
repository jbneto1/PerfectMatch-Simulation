set(LOGGER_SRC
        logger.cpp
        )

add_library(Logger ${LOGGER_SRC})

target_include_directories(Logger PRIVATE
                        ${PROJECT_SOURCE_DIR}/include/third_party/)