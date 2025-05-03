cmake_minimum_required(VERSION 3.30)

project(foc)

add_library(foc INTERFACE)

target_sources(foc INTERFACE FOC.cpp)

target_include_directories(foc INTERFACE
        ./
        ./base_classes
)