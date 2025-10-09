cmake_minimum_required(VERSION 3.28)

project(rmdev_ins C CXX)

add_library(${PROJECT_NAME} STATIC)

target_link_libraries(${PROJECT_NAME}
    PUBLIC emdevif_core
    PRIVATE CMSISDSP rmdev_device_model
)
