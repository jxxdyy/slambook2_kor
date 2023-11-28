# Declare the minimum version of cmake required
cmake_minimum_required(VERSION 2.8)

# Declare a cmake project
project(HelloSLAM)

# Set Compilation Mode
set(CMAKE_BUILD_TYPE "Debug")

# Add an executable
# Syntax: add_executable(program name source code file)
add_executable(helloSLAM helloSLAM.cpp)


#! ------------------------------------------------------
# Add Hello Library
add_library(hello libHelloSLAM.cpp)
# shared library
add_library(hello_shared SHARED libHelloSLAM.cpp)

# Add executable to call functions in hello library
add_executable(useHello useHello.cpp)
# Link library files to executables
target_link_libraries(useHello hello_shared)
