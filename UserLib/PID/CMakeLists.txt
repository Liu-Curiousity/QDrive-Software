cmake_minimum_required(VERSION 3.30)

project(PID)

add_library(PID INTERFACE)

target_sources(PID INTERFACE)

target_include_directories(PID INTERFACE
        ./
)