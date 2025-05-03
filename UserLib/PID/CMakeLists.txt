cmake_minimum_required(VERSION 3.30)

project(pid)

add_library(pid INTERFACE)

target_sources(pid INTERFACE PID.cpp)

target_include_directories(pid INTERFACE
        ./
)