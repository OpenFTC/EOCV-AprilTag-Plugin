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
#include <android/log.h>
#include "stdexcept"

const char* LOG_TAG = "AprilTagDetectorJNI";

class AprilTagFamily
{
private:
    const char* const FAMILY_36h11 = "tag36h11";
    const char* const FAMILY_25h9 = "tag25h9";
    const char* const FAMILY_16h5 = "tag16h5";
    const char* const FAMILY_41h12 = "tagStandard41h12";

public:
    apriltag_family_t* tf = NULL;

public:
    AprilTagFamily(const char* const familyName)
    {
        if (!strcmp(familyName, FAMILY_36h11))
        {
            __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Creating 36h11 tag family");
            tf = tag36h11_create();
        }
        else if (!strcmp(familyName, FAMILY_25h9))
        {
            __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Creating 25h9 tag family");
            tf = tag25h9_create();
        }
        else if (!strcmp(familyName, FAMILY_16h5))
        {
            __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Creating 16h5 tag family");
            tf = tag16h5_create();
        }
        else if (!strcmp(familyName, FAMILY_41h12))
        {
            __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Creating Standard41h12 tag family");
            tf = tagStandard41h12_create();
        }
        else
        {
            throw std::invalid_argument("Unsupported tag family name. Use e.g. \"tag36h11\".");
        }
    }

    ~AprilTagFamily()
    {
        __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Freeing tag family %s", tf->name);

        if (!strcmp(tf->name, FAMILY_36h11))
        {
            tag36h11_destroy(tf);
        }
        else if (!strcmp(tf->name, FAMILY_25h9))
        {
            tag25h9_destroy(tf);
        }
        else if (!strcmp(tf->name, FAMILY_16h5))
        {
            tag16h5_destroy(tf);
        }
        else if (!strcmp(tf->name, FAMILY_41h12))
        {
            tagStandard41h12_destroy(tf);
        }
        else
        {
            __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "FAILED to free unknown tag family %s", tf->name);
        }
    }
};

class AprilTagCtx
{
    AprilTagFamily tf;

public:
    apriltag_detector_t* td;

public:
    AprilTagCtx(const char* const familyName, int threads, float decimate) : tf(familyName)
    {
        __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Initializing april tag detector");
        td = apriltag_detector_create();
        td->nthreads = threads;
        td->quad_decimate = decimate;
        apriltag_detector_add_family(td,tf.tf);
    }

    ~AprilTagCtx()
    {
        __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Freeing april tag detector");
        apriltag_detector_destroy(td);
    }
};

extern "C" JNIEXPORT jlong JNICALL
Java_org_openftc_apriltag_AprilTagDetectorJNI_runApriltagDetector(JNIEnv *env, jclass clazz, jlong jPtrContext, jlong ptrGreyscaleBuf, jint width, jint height)
{
    AprilTagCtx* context = (AprilTagCtx*) jPtrContext;

    if(context == NULL || ptrGreyscaleBuf == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return 0;
    }

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {
            .width = width,
            .height = height,
            .stride = width,
            .buf = (uint8_t*) ptrGreyscaleBuf
    };

    zarray_t* detections = apriltag_detector_detect(context->td, &im);

    // If the array is empty, we're not going to return a pointer to it to
    // user code, so go ahead and release it now (otherwise it's a memory leak)
    if(zarray_size(detections) == 0)
    {
        zarray_destroy(detections);
        return 0;
    }
    else
    {
        return (jlong ) detections;
    }
}

extern "C" JNIEXPORT void JNICALL
Java_org_openftc_apriltag_AprilTagDetectorJNI_setApriltagDetectorDecimation(JNIEnv *env, jclass clazz, jlong jPtrContext, jfloat decimate)
{
    AprilTagCtx* context = (AprilTagCtx*) jPtrContext;

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

    try
    {
        AprilTagCtx* ptrContext = new AprilTagCtx(famname, threads, decimate);
        return (jlong) ptrContext;
    }
    catch (std::invalid_argument& e)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                e.what());
    }

    env->ReleaseStringUTFChars(jfamname, famname);
    return 0;
}

extern "C" JNIEXPORT void JNICALL
Java_org_openftc_apriltag_AprilTagDetectorJNI_releaseApriltagDetector(JNIEnv *env, jclass clazz, jlong ptrContext)
{
    AprilTagCtx* context = (AprilTagCtx*) ptrContext;

    if(context == NULL)
    {
        env->ThrowNew(
                env->FindClass("java/lang/IllegalArgumentException"),
                "Pointer must not be null!");
        return;
    }

    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Freeing april tag JNI context object");
    delete context;
    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "Freed april tag JNI context object");
}