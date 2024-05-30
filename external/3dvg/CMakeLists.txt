cmake_minimum_required(VERSION 3.8)
project(visibility_graph)

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} 
    src/visibility.cpp
    VisiLibity1/src/visilibity.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    src
    VisiLibity1/src
    ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
)