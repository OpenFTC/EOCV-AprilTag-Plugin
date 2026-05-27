# EOCV-AprilTag-Plugin

A plug and play module for detecting AprilTags on an FTC robot, designed to be used from EasyOpenCV

## Installation instructions:

This library is now packaged in the official upstream SDK! No need to install anything manually.

## AprilTag images

You can download a PDF with the first 20 tags of the 36h11 family [here](https://www.dotproduct3d.com/uploads/8/5/1/1/85115558/apriltags1-20.pdf)

Alternatively, you can find PNGs [here](https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11)

## Changelog:

### v2.2.0

- Updates to the latest version of AprilTag as of 25 May 2026

### v2.1.0

- Updates to the latest version of AprilTag as of 8 June 2024

### v2.0.0

 - **API CHANGE:** `AprilTagPose` now provides the full 3x3 rotation matrix rather than precomputed euler angles. You can get euler angles the same way as with the IMU; i.e. `Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);`
 - Updates to the latest version of AprilTag as of 6 May 2023

### v1.1.1

 - Fixes OpenCV-Repackaged dependency to be `compileOnly`
 - Removes need to link against OpenCV native library
 - Fixes buffer overrun when creating JNI detector context
 - Prints library version in static initializer

### v1.1.0

 - Updates to latest version of AprilTag as of 30 March 2022
 - Fixes small memory leak for frames where no detections were found
 - Protects against NULL pointers in native JNI code
 - Updates `AprilTagDetectionPipeline` to create the native detector in the constructor instead of in init(). NOTE: You will need to copy-paste the updated file, as this file is not part of the binary release!

### v1.0.0

 - Initial release
