set(OpenCV_DIR /usr/local/include/build_opencv/unix-install)
# set(OpenCV_DIR /usr/local/include/opencv)

cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

# set(OpenCV_DIR /Users/lyh/Desktop/Documents/opencv/build_opencv/unix-install)
# list(APPEND CMAKE_PREFIX_PATH "/usr/local/include/opencv/")
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
