cmake_minimum_required(VERSION 3.28...4.0.2)

project(rmdev_ins C CXX)

add_library(${PROJECT_NAME} STATIC)

target_link_libraries(${PROJECT_NAME}
    PUBLIC emdevif_core
    PRIVATE emdevif_compile_options rmdev_math rmdev_device_model emdevif_timeline CMSISDSP
)

target_include_directories(${PROJECT_NAME} PUBLIC inc)
target_sources(${PROJECT_NAME} PRIVATE src/quaternion_ekf_ins/kalman_filter.cpp)

if (EMDEVIF_USE_CPP_MODULES)
    target_sources(${PROJECT_NAME} PUBLIC FILE_SET ${PROJECT_NAME}_modules TYPE CXX_MODULES
        FILES ins.cppm
    )
endif ()
