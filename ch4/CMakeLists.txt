cmake_minimum_required(VERSION 3.0)
project(useSophus)

# Sophus
find_package(Sophus REQUIRED)

# Eigen
include_directories("/usr/include/eigen3")
add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)

add_subdirectory(example)
