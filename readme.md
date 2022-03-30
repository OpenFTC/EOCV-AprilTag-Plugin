# EOCV-AprilTag-Plugin

A plug and play module for detecting AprilTags on an FTC robot, designed to be used from EasyOpenCV

## Installation instructions:

**IMPORTANT NOTE: This assumes you have already installed EasyOpenCV!**

1. Open your FTC SDK Android Studio project

2. Open the `build.gradle` file for the TeamCode module:

    ![img-here](doc/images/teamcode-gradle.png)

3. At the bottom, add this:

        dependencies {
            implementation 'org.openftc:apriltag:1.1.0'
         }

4. Now perform a Gradle Sync:

    ![img-here](doc/images/gradle-sync.png)

5. Congrats, you're ready to go! Now check out the example OpModes and pipeline in the [examples](https://github.com/OpenFTC/EOCV-AprilTag-Plugin/tree/master/examples/src/main/java/org/firstinspires/ftc/teamcode) directory.

## AprilTag images

You can download a PDF with the first 20 tags of the 36h11 family [here](https://www.dotproduct3d.com/uploads/8/5/1/1/85115558/apriltags1-20.pdf)

Alternatively, you can find PNGs [here](https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11)

## Changelog:

### v1.1.0

 - Updates to latest version of AprilTag as of 30 March 2022
 - Fixes small memory leak for frames where no detections were found
 - Protects against NULL pointers in native JNI code
 - Updates `AprilTagDetectionPipeline` to create the native detector in the constructor instead of in init(). NOTE: You will need to copy-paste the updated file, as this file is not part of the binary release!

### v1.0.0

 - Initial release
