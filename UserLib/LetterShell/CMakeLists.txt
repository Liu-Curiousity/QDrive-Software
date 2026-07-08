cmake_minimum_required(VERSION 3.30)

project(LetterShell)

add_library(LetterShell OBJECT
        ./Src/shell.c
        ./Src/shell_cmd_list.c
        ./Src/shell_companion.c
        ./Src/shell_ext.c
)

target_include_directories(LetterShell PUBLIC
        ./Inc
)
