cmake_minimum_required(VERSION 3.15)
project(CLionProject)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fexec-charset=GBK")
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(CLionProject main.cpp)
file(GLOB sources CONFIGURE_DEPENDS *.cpp *.hpp *.h)
target_sources(CLionProject PUBLIC ${sources})

set(OpenCV_DIR C:/opencv-4.x/opencv_build/install)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(CLionProject PUBLIC ${OpenCV_LIBS})

set(Eigen3_DIRS "C:/ProgramFiles(x86)/Eigen3/share/eigen3/cmake")
find_package(Eigen3 REQUIRED)
# include_directories(C:/ProgramFiles(x86)/Eigen3/include/eigen3/Eigen) # ./Eigen3/include/eigen3/Eigen
target_link_libraries(CLionProject PUBLIC Eigen3::Eigen)