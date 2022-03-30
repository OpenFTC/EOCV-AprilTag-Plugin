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

#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <jni.h>
#include <apriltag.h>
#include <tag36h11.h>
#include <tag25h9.h>
#include <tag16h5.h>
#include <tagStandard41h12.h>
#include <common/getopt.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/core.hpp>
#include <android/log.h>

const char* LOG_TAG = "AprilTagDetectorJNI";

struct ApriltagDetectorJniContext
{
    apriltag_family_t *tf = NULL;
    apriltag_detector_t *td = NULL;
    char* tagType = NULL;
};

extern "C" JNIEXPORT jlong JNICALL
Java_org_openftc_apriltag_AprilTagDetectorJNI_runApriltagDetector(JNIEnv *env, jclass clazz, jlong jPtrContext, jlong jPtrGreyscaleMat)
{
    cv::Mat* grey  = (cv::Mat*) jPtrGreyscaleMat;
    ApriltagDetectorJniContext* context = (ApriltagDetectorJniContext*) jPtrContext;

    if(context == NULL || grey == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return 0;
    }

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {
            .width = grey->cols,
            .height = grey->rows,
            .stride = grey->cols,
            .buf = grey->data
    };

    zarray_t* detections = apriltag_detector_detect(context->td, &im);
    int numDetections = zarray_size(detections);

    if(numDetections == 0)
    {
        return 0;
    }

    return (jlong ) detections;
}

extern "C" JNIEXPORT void JNICALL
Java_org_openftc_apriltag_AprilTagDetectorJNI_setApriltagDetectorDecimation(JNIEnv *env, jclass clazz, jlong jPtrContext, jfloat decimate)
{
    ApriltagDetectorJniContext* context = (ApriltagDetectorJniContext*) jPtrContext;

    if(context == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return;
    }

    context->td->quad_decimate = decimate;
}

extern "C" JNIEXPORT jlong JNICALL
Java_org_openftc_apriltag_AprilTagDetectorJNI_createApriltagDetector(JNIEnv *env, jclass clazz, jstring jfamname, jfloat decimate, jint threads)
{
    const char *famname = env->GetStringUTFChars(jfamname, nullptr);

    apriltag_family_t *tf = NULL;

    if (!strcmp(famname, "tag36h11"))
    {
        __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Creating 36h11 tag family");
        tf = tag36h11_create();
    }
    else if (!strcmp(famname, "tag25h9"))
    {
        __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Creating 25h9 tag family");
        tf = tag25h9_create();
    }
    else if (!strcmp(famname, "tag16h5"))
    {
        __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Creating 16h5 tag family");
        tf = tag16h5_create();
    }
    else if (!strcmp(famname, "tagStandard41h12"))
    {
        __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Creating Standard41h12 tag family");
        tf = tagStandard41h12_create();
    }
    else
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Unrecognized tag family name. Use e.g. \"tag36h11\".");

        return 0;
    }

    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Initializing april tag detector");
    apriltag_detector_t* td = apriltag_detector_create();
    td->nthreads = threads;
    td->quad_decimate = decimate;
    //td->debug = true;
//    td->quad_sigma = blur;
//    td->refine_edges = refineEdges;

    apriltag_detector_add_family(td, tf);

    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Initializing april tag JNI context object");
    ApriltagDetectorJniContext* ptrContext = new ApriltagDetectorJniContext();
    ptrContext->tf = tf;
    ptrContext->td = td;
    char *strForContext = new char[strlen(famname)];
    strcpy(strForContext, famname);
    ptrContext->tagType = strForContext;

    env->ReleaseStringUTFChars(jfamname, famname);

    jlong ret = (jlong) ptrContext;

    return ret;
}

extern "C" JNIEXPORT void JNICALL
Java_org_openftc_apriltag_AprilTagDetectorJNI_releaseApriltagDetector(JNIEnv *env, jclass clazz, jlong ptrContext)
{
    ApriltagDetectorJniContext* bridge = (ApriltagDetectorJniContext*) ptrContext;

    if(bridge == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return;
    }

    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Freeing april tag detector");
    apriltag_detector_destroy(bridge->td);

    char *famname = bridge->tagType;
    apriltag_family_t *tf = bridge->tf;

    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Freeing tag family");

    if (!strcmp(famname, "tag36h11"))
    {
        tag36h11_destroy(tf);
    }
    else if (!strcmp(famname, "tag25h9"))
    {
        tag25h9_destroy(tf);
    }
    else if (!strcmp(famname, "tag16h5"))
    {
        tag16h5_destroy(tf);
    }
    else if (!strcmp(famname, "tagStandard41h12"))
    {
        tagStandard41h12_destroy(tf);
    }

    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Freeing tag type");
    delete[] bridge->tagType;

    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Freeing april tag JNI context object");
    delete bridge;
}