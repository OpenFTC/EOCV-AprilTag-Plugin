/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <jni.h>
#include <apriltag.h>
#include <tag36h11.h>
#include <tag25h9.h>
#include <tag16h5.h>
#include <tagStandard41h12.h>
#include <common/getopt.h>
#include <apriltag_pose.h>
#include <iostream>
#include <android/log.h>

extern "C" JNIEXPORT jint JNICALL
Java_org_openftc_apriltag_ApriltagDetectionJNI_getId(JNIEnv *env, jclass clazz, jlong ptr)
{
    apriltag_detection_t* detection = (apriltag_detection*) ptr;

    if(detection == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return NULL;
    }

    return detection->id;
}

extern "C" JNIEXPORT jint JNICALL
Java_org_openftc_apriltag_ApriltagDetectionJNI_getHamming(JNIEnv *env, jclass clazz, jlong ptr)
{
    apriltag_detection_t* detection = (apriltag_detection*) ptr;

    if(detection == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return NULL;
    }

    return detection->hamming;
}

extern "C" JNIEXPORT jfloat JNICALL
Java_org_openftc_apriltag_ApriltagDetectionJNI_getDecisionMargin(JNIEnv *env, jclass clazz, jlong ptr)
{
    apriltag_detection_t* detection = (apriltag_detection*) ptr;

    if(detection == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return NULL;
    }

    return detection->decision_margin;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_org_openftc_apriltag_ApriltagDetectionJNI_getCenterpoint(JNIEnv *env, jclass clazz, jlong ptr)
{
    apriltag_detection_t* detection = (apriltag_detection*) ptr;

    if(detection == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return NULL;
    }

    jdoubleArray result = env->NewDoubleArray(2);
    env->SetDoubleArrayRegion(result, 0, 2, detection->c);

    return result;
}

extern "C" JNIEXPORT jdouble JNICALL
Java_org_openftc_apriltag_ApriltagDetectionJNI_getSize(JNIEnv *env, jobject type, jlong ptr)
{
    apriltag_detection_t* detection = (apriltag_detection*) ptr;

    if(detection == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return 0;
    }

    double* pointTopLeft = detection->p[0];
    double* pointBottomLeft = detection->p[1];
    double* pointBottomRight = detection->p[2];
    double* pointTopRight = detection->p[3];

    int x = 0;
    int y = 1;

    double dimA = hypot(pointTopLeft[x]-pointTopRight[x], pointTopLeft[y]-pointTopRight[y]);
    double dimB = hypot(pointTopLeft[x]-pointBottomLeft[x], pointTopLeft[y]-pointBottomLeft[y]);

    return sqrt(dimA*dimB);
}

extern "C" JNIEXPORT jobjectArray JNICALL
Java_org_openftc_apriltag_ApriltagDetectionJNI_getCorners(JNIEnv *env, jclass clazz, jlong ptr)
{
    apriltag_detection_t* detection = (apriltag_detection*) ptr;

    if(detection == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return NULL;
    }

    // Get the length for the first and second dimensions
    unsigned int lengthDim1 = 4; //4 corners
    unsigned int lengthDim2 = 2; //x,y

    // Get the double array class
    jclass doubleArrayClass = env->FindClass("[D");

    // Check if we properly got the double array class
    if (doubleArrayClass == NULL)
    {
        // Ooops
        return NULL;
    }

    // Create the returnable 2D array
    jobjectArray java2dDoubleArray = env->NewObjectArray((jsize) lengthDim1, doubleArrayClass, NULL);

    // Go through the first dimension and add the second dimension arrays
    for (int i = 0; i < lengthDim1; i++)
    {
        jdoubleArray doubleArray = env->NewDoubleArray(lengthDim2);
        env->SetDoubleArrayRegion(doubleArray, (jsize) 0, (jsize) lengthDim2, detection->p[i]);
        env->SetObjectArrayElement(java2dDoubleArray, (jsize) i, doubleArray);
        env->DeleteLocalRef(doubleArray);
    }

    // Return a Java consumable 2D double array
    return java2dDoubleArray;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_org_openftc_apriltag_ApriltagDetectionJNI_getPoseEstimate(JNIEnv *env, jclass clazz, jlong ptr,
                                                      jdouble tag_size, jdouble fx, jdouble fy, jdouble cx, jdouble cy)
{
    apriltag_detection_t* det = (apriltag_detection*) ptr;

    if(det == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return NULL;
    }

    apriltag_detection_info_t info;
    info.det = det;
    info.tagsize = tag_size;
    info.fx = fx;
    info.fy = fy;
    info.cx = cx;
    info.cy = cy;

    // Then call estimate_tag_pose.
    apriltag_pose_t pose;
    //double err = estimate_tag_pose(&info, &pose);
    estimate_tag_pose(&info, &pose);

    jdoubleArray result = env->NewDoubleArray(3 + 3*3);
    //__android_log_print(ANDROID_LOG_DEBUG, "APRIL", "Pose: X=%.2f Y=%.2f Z=%.2f", pose.t->data[0], pose.t->data[1], pose.t->data[2]);
    env->SetDoubleArrayRegion(result, 0, 3 /* 0->2  */, pose.t->data);
    env->SetDoubleArrayRegion(result, 3, 9 /* 3->8  */, pose.R->data);

    return result;
}

extern "C" JNIEXPORT jlongArray JNICALL
Java_org_openftc_apriltag_ApriltagDetectionJNI_getDetectionPointers(JNIEnv *env, jclass clazz, jlong ptr)
{
    zarray_t* detections = (zarray_t*) ptr;

    if(detections == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return NULL;
    }

    int numDetections = zarray_size(detections);

    if(numDetections == 0)
    {
        return NULL;
    }

    jlong detectionPointers[numDetections];

    for(int i = 0; i < numDetections; i++)
    {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        detectionPointers[i] = (jlong) det;
    }

    jlongArray result = env->NewLongArray(numDetections);
    env->SetLongArrayRegion(result, 0, numDetections, detectionPointers);

    return result;
}

extern "C"
JNIEXPORT void JNICALL
Java_org_openftc_apriltag_ApriltagDetectionJNI_freeDetectionList(JNIEnv *env, jclass clazz, jlong jPtrDetections)
{
    zarray_t* detections = (zarray_t*) jPtrDetections;

    if(detections == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return;
    }

    // Destroys the array AND all the detections inside
    apriltag_detections_destroy(detections);
}
