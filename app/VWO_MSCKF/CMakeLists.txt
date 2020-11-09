cmake_minimum_required(VERSION 2.8.8)
project(VWO_MSCKF)

# Set C++14
add_compile_options(-std=c++14)

# Set cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Third party dependencies
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Glog  REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}/opencv4)

# TinyGrapeKit library
set(TGK_DIR ${CMAKE_SOURCE_DIR}/../../library/install)
include_directories(${TGK_DIR}/include)
link_directories(${TGK_DIR}/lib)

# Make library
file(GLOB_RECURSE ALL_HEADER_FILES
    ${PROJECT_SOURCE_DIR}/include/*.h
)
file(GLOB_RECURSE ALL_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)
include_directories(include)

#add_library(${PROJECT_NAME} SHARED ${ALL_SRC_FILES} ${ALL_HEADER_FILES})
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

## Declare C++ executables
add_executable(RunKAISTData example/RunKAISTData.cpp)
target_link_libraries(RunKAISTData glog ${OpenCV_LIBS} TinyGrapeKit)