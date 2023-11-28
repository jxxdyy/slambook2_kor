
cmake_minimum_required(VERSION 3.1)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17 -O2")

project(imageBasics)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS} 
)

add_executable(imageBasics imageBasics.cpp)
# OpenCV 라이브러리 link
target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)
# OpenCV 라이브러리 link
target_link_libraries(undistortImage ${OpenCV_LIBS})
