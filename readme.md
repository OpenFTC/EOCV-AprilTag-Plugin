# EOCV-AprilTag-Plugin

A plug and play module for detecting AprilTags on an FTC robot, designed to be used from EasyOpenCV

## Installation instructions:

**IMPORTANT NOTE: This tutorial assumes you have already set up EasyOpenCV in your workspace**

1. Open your FTC SDK Android Studio project

2. Open the `build.gradle` file for the TeamCode module:

    ![img-here](doc/images/teamcode-gradle.png)

3. At the bottom, add this:

        dependencies {
            implementation 'org.openftc:apriltag:1.0.0'
         }

4. Now perform a Gradle Sync:

    ![img-here](doc/images/gradle-sync.png)

5. Congrats, you're ready to go! Now check out the example OpModes and pipeline in the examples directory.

## Changelog:

### v1.0.0

 - Initial release