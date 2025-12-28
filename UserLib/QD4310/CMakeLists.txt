cmake_minimum_required(VERSION 3.30)

project(qd4310)

add_library(qd4310 INTERFACE)

target_sources(qd4310 INTERFACE QD4310.cpp)

target_include_directories(qd4310 INTERFACE
        ./
)