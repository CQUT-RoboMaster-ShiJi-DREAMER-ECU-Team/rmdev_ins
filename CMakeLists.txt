cmake_minimum_required(VERSION 3.28)

project(rmdev_ins C CXX)

add_library(${PROJECT_NAME} STATIC)

target_link_libraries(${PROJECT_NAME}
    PUBLIC emdevif_core
    PRIVATE emdevif_compile_options CMSISDSP rmdev_device_model
)

target_sources(${PROJECT_NAME} PUBLIC
    FILE_SET ${PROJECT_NAME}_modules TYPE CXX_MODULES FILES
        ins.cppm ins_base.cppm
        quaternion_ekf_ins/QuaternionEKF_INS.cppm

    PRIVATE quaternion_ekf_ins/kalman_filter.c
    PRIVATE FILE_SET ${PROJECT_NAME}_headers TYPE HEADERS
        BASE_DIRS quaternion_ekf_ins/ FILES quaternion_ekf_ins/kalman_filter.h
)
