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
             src/main/cpp/apriltag_common/src/g2d.c
             src/main/cpp/apriltag_common/src/getopt.c
             src/main/cpp/apriltag_common/src/homography.c
             src/main/cpp/apriltag_common/src/image_u8.c
             src/main/cpp/apriltag_common/src/image_u8x3.c
             src/main/cpp/apriltag_common/src/image_u8x4.c
             src/main/cpp/apriltag_common/src/matd.c
             src/main/cpp/apriltag_common/src/pam.c
             src/main/cpp/apriltag_common/src/pjpeg.c
             src/main/cpp/apriltag_common/src/pjpeg-idct.c
             src/main/cpp/apriltag_common/src/pnm.c
             src/main/cpp/apriltag_common/src/string_util.c
             src/main/cpp/apriltag_common/src/svd22.c
             src/main/cpp/apriltag_common/src/time_util.c
             src/main/cpp/apriltag_common/src/unionfind.c
             src/main/cpp/apriltag_common/src/workerpool.c
             src/main/cpp/apriltag_common/src/zarray.c
             src/main/cpp/apriltag_common/src/zhash.c
             src/main/cpp/apriltag_common/src/zmaxheap.c
             src/main/cpp/apriltag/src/apriltag.c
             src/main/cpp/apriltag/src/apriltag_pose.c
             src/main/cpp/apriltag/src/apriltag_quad_thresh.c
             src/main/cpp/apriltag/src/tag16h5.c
             src/main/cpp/apriltag/src/tag25h9.c
             src/main/cpp/apriltag/src/tag36h11.c
             src/main/cpp/apriltag/src/tagCircle21h7.c
             src/main/cpp/apriltag/src/tagStandard41h12.c)

# VROOOOOOOM
target_compile_options(apriltag PRIVATE
        "$<$<CONFIG:RELEASE>:-O3>"
        "$<$<CONFIG:DEBUG>:-O3>"
        )

# Specifies a path to native header files.
include_directories(
                     src/main/cpp/apriltag/include
                     src/main/cpp/apriltag_common/include
)

# linking libs
target_link_libraries(
        apriltag
        log
        android)