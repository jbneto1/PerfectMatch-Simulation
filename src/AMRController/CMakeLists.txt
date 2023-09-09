set(AMR_CONTROLLER_SRC
        AMRController.cpp
        MecanumKinematics/MecanumKinematics.cpp
        )

add_library(AMRController ${AMR_CONTROLLER_SRC})

target_include_directories(AMRController PRIVATE
                        ${PROJECT_SOURCE_DIR}/src/
                        ${PROJECT_SOURCE_DIR}/include/
                        ${PROJECT_SOURCE_DIR}/include/third_party
                        ${PROJECT_SOURCE_DIR}/include/third_party/eigen-3.4.0/)
