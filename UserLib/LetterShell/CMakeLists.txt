cmake_minimum_required(VERSION 3.30)

project(letter_shell)

add_library(letter_shell INTERFACE)

target_compile_definitions(letter_shell INTERFACE SHELL_CFG_USER="shell_cfg_user.h")

target_sources(letter_shell INTERFACE
        ./Src/shell.c
        ./Src/shell_cmd_list.c
        ./Src/shell_companion.c
        ./Src/shell_ext.c
)

target_include_directories(letter_shell INTERFACE
        ./Inc
)