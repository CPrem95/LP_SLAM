# CMakeLists.txt for RCM sample app for Linux

# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Project name
project(rcmSampleApp)

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

# Source files
set(SOURCES
    rcmSampleApp.c
    rcm.c
    rcmIf.c
)

# Executable target
add_executable(rcmSampleApp ${SOURCES})