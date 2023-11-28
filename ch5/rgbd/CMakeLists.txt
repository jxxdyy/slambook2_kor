
cmake_minimum_required(VERSION 3.1)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17 -O2")

project(stereoVision)

find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    ${Sophus_INCLUDE_DIRS}
)

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES})