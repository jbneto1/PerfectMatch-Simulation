set(MANAGER_SRC
        Manager.cpp
        )

add_library(Manager ${MANAGER_SRC})

target_include_directories(Manager PRIVATE
                        ${PROJECT_SOURCE_DIR}/src/
                        ${PROJECT_SOURCE_DIR}/include/
                        ${PROJECT_SOURCE_DIR}/include/third_party/
                        ${PROJECT_SOURCE_DIR}/include/third_party/standalone_asio/
                        ${PROJECT_SOURCE_DIR}/include/third_party/eigen-3.4.0/)
