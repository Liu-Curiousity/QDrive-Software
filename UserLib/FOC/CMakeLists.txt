cmake_minimum_required(VERSION 3.30)

project(foc)

add_library(foc INTERFACE)

target_sources(foc INTERFACE core/FOC.cpp Stroage_EmbeddedFlash.cpp)

target_include_directories(foc INTERFACE
        ./
        core
        core/base_classes
)