project(reconstruct)
cmake_minimum_required (VERSION 2.8)

#opencv 
set(OpenCV_INSTALL_PATH "/home/jxie/jun/software/OPENCV_INSTALL")
find_package(OpenCV 2.4.9 REQUIRED PATHS ${OpenCV_INSTALL_PATH})

#find_package(OpenCV 2.4.9 REQUIRED)

#pcl
find_package(PCL 1.7 REQUIRED)

SET(CMAKE_BUILD_TYPE "RELEASE")

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable( reconstruct src/scan.cpp src/read.cpp src/registration.cpp src/common.cpp)
target_link_libraries (reconstruct ${PCL_LIBRARIES} ${OpenCV_LIBS} )

