# Sets the minimum version of CMake required to build your native library.
# This ensures that a certain set of CMake features is available to
# your build.

cmake_minimum_required(VERSION 3.4.1)

set (CMAKE_CXX_STANDARD 11)

# Specifies a library name, specifies whether the library is STATIC or
# SHARED, and provides relative paths to the source code. You can
# define multiple libraries by adding multiple add_library() commands,
# and CMake builds them for you. When you build your app, Gradle
# automatically packages shared libraries with your APK.

add_library( # Specifies the name of the library.
             apriltag

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             src/main/cpp/AprilTagDetectorJNI.cpp
             src/main/cpp/AprilTagDetectionJNI.cpp
        
             apriltag/common/g2d.c
             apriltag/common/getopt.c
             apriltag/common/homography.c
             apriltag/common/image_u8.c
             apriltag/common/image_u8x3.c
             apriltag/common/image_u8x4.c
             apriltag/common/matd.c
             apriltag/common/pam.c
             apriltag/common/pjpeg.c
             apriltag/common/pjpeg-idct.c
             apriltag/common/pnm.c
             apriltag/common/string_util.c
             apriltag/common/svd22.c
             apriltag/common/time_util.c
             apriltag/common/unionfind.c
             apriltag/common/workerpool.c
             apriltag/common/zarray.c
             apriltag/common/zhash.c
             apriltag/common/zmaxheap.c
             apriltag/apriltag.c
             apriltag/apriltag_pose.c
             apriltag/apriltag_quad_thresh.c
             apriltag/tag16h5.c
             apriltag/tag25h9.c
             apriltag/tag36h11.c
             apriltag/tagCircle21h7.c
             apriltag/tagStandard41h12.c)

# VROOOOOOOM
target_compile_options(apriltag PRIVATE
        "$<$<CONFIG:RELEASE>:-O3>"
        "$<$<CONFIG:DEBUG>:-O3>"
        )

# Specifies a path to native header files.
include_directories(
                     apriltag
                     apriltag/common
)

# linking libs
target_link_libraries(
        apriltag
        log
        android)