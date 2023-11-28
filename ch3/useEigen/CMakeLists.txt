cmake_minimum_required(VERSION 3.1)
project(useEigen)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

# Eigen 헤더 파일 추가
include_directories("/usr/include/eigen3")
add_executable(eigenMatrix eigenMatrix.cpp)
