cmake_minimum_required(VERSION 3.22)
project(VirtualTouchPad)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_library(native-lib SHARED native-lib.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# OpenCV와 TBB의 .so 파일 경로
set(OPENCV_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../jniLibs/${ANDROID_ABI})

add_library(opencv_world SHARED IMPORTED)
set_target_properties(opencv_world PROPERTIES
        IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_world.so
)

add_library(tbb SHARED IMPORTED)
set_target_properties(tbb PROPERTIES
        IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libtbb.so
)

find_library(log-lib log)
find_library(jnigraphics-lib jnigraphics)

target_link_libraries(native-lib
        opencv_world
        tbb
        ${log-lib}
        ${jnigraphics-lib}
)
