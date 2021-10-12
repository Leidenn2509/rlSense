cmake_minimum_required(VERSION 3.17)
project(rsTest)

set(CMAKE_CXX_STANDARD 20)

# Find librealsense2 installed package
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(rsTest main.cpp)

# Link librealsense2 to the target
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY} ${OpenCV_LIBS})