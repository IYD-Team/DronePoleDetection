cmake_minimum_required(VERSION 2.8)
project( PoleDetection )
find_package( OpenCV REQUIRED )

add_executable( main Source.cpp )
target_link_libraries( main ${OpenCV_LIBS} )
