/*
 * @file implement nuc2p
 * @author Le Duy Nhat
 * @date Nov 10th 2021
 */
//==============================================================================
// Include Files
//==============================================================================
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <stdlib.h>
#include <android/log.h>
#include <fastcv/fastcv.h>

#include <string.h>
#include "utils/FastCVUtil.h"
#include "PreProcessing.h"
#include "utils/FastCVSampleRenderer.h"
#include "utils.h"

//==============================================================================
// Declarations
//==============================================================================
typedef enum PREPROCESS
{
    NO_PREPROCESS,
    NUC_PROCESS,
    GAUSE
};

//------------------------------------------------------------------------------
/// @brief
///    Contains state information of the instance of the application.
//------------------------------------------------------------------------------
struct PreProcessingState
{
    //---------------------------------------------------------------------------
    /// Constructor for preprocessing object sets variables to default values.
    //---------------------------------------------------------------------------
    PreProcessingState()
    {
        preProcessImgBuf = NULL;

        alignedImgBuf = NULL;
        alignedImgWidth = 0;
        alignedImgHeight = 0;
        preProcessType = NO_PREPROCESS;
    }
    /// Image buffer that stores 8 bit preprocessing image
    uint8_t*                   preProcessImgBuf;

    /// Image buffer that stores 16 bit preprocessing image
    uint16_t*                  u16preProcessImgBuff;

    /// Image buffer that stores 32 bit preprocessing image
    uint32_t*                  u32preProcessImgBuff;

    /// Image buffer that stores float 32 bit preprocessing image
    float32_t*                  f32preProcessImgBuff;

    /// Width of image
    uint32_t                   preProcessImgWidth;

    /// Height of image
    uint32_t                   preProcessImgHeight;

    /// Buffer to place image data if camera callback data is not aligned
    uint8_t*                   alignedImgBuf;

    /// Width of aligned image
    uint32_t                   alignedImgWidth;

    /// Height of aligned image
    uint32_t                   alignedImgHeight;

    /**
     *  Others
     */
    /// parameter buffers from config files
    uint8_t*                    DeadPixel;
    float32_t*                  Offset;
    float32_t*                  Gain_correction;
    float32_t*                  OffsetFFC;

    /// Affine type selected by user
    PREPROCESS                 preProcessType;
};

/// Application' state structure to hold global state for sample app.
static PreProcessingState     preProcessingState;

//==============================================================================
// Function Definitions
//==============================================================================
extern "C" JNIEXPORT void JNICALL Java_com_qualcomm_fastcvdemo_apis_preprocessing_PreProcessing_setOption
        (
                JNIEnv*  env,
                jobject  obj,
                int opts
        )
{
    preProcessingState.preProcessType = (PREPROCESS) opts;

    if(preProcessingState.preProcessImgBuf)
    {
        memset(preProcessingState.preProcessImgBuf, 0, preProcessingState.alignedImgWidth * preProcessingState.alignedImgHeight);
        memset(preProcessingState.preProcessImgBuf+(preProcessingState.preProcessImgWidth*preProcessingState.preProcessImgHeight),
               128,
               preProcessingState.preProcessImgWidth*preProcessingState.preProcessImgHeight/2);
    }
}
//------------------------------------------------------------------------------
/// @brief Calls FastCV API to find Corners
///        Additionally this function overlays pixels on the camera preview
///        image
/// @param img Pointer to camera image
/// @param w Width of image
/// @param y Height of height
//------------------------------------------------------------------------------
extern "C" JNIEXPORT void JNICALL Java_com_qualcomm_fastcvdemo_apis_preprocessing_PreProcessing_update
(
    JNIEnv*     env,
    jobject     obj,
    jbyteArray  img,
    jint        w,
    jint        h
    )
{
    jbyte*            jimgData = NULL;
    jboolean          isCopy = 0;
    uint8_t*          renderBuffer;
    uint64_t          time;
    float             timeMs;
    // Allocate the buffer once here if it's not allocated already
    if( preProcessingState.preProcessImgBuf == NULL)
    {
        int frameSize = w*h*3/2;
        preProcessingState.preProcessImgBuf = (uint8_t *)fcvMemAlloc(frameSize, 16);
        if( preProcessingState.preProcessImgBuf == NULL )
        {
            EPRINTF("Allocate affineImgBuf failed");
            return;
        }
        else
        {
            memset(preProcessingState.preProcessImgBuf, 0, w*h);
            memset(preProcessingState.preProcessImgBuf+(w*h), 128, w*h/2);
        }
    }
    // Get data from JNI
    jimgData = env->GetByteArrayElements( img, &isCopy );

    renderBuffer = getRenderBuffer( w, h );

    lockRenderBuffer();

    time = util.getTimeMicroSeconds();

    // jimgData might not be 128 bit aligned.
    // fcvColorYUV420toRGB565u8() and other fcv functionality inside
    // require 128 bit memory aligned. In case of jimgData
    // is not 128 bit aligned, it will allocate memory that is 128 bit
    // aligned and copies jimgData to the aligned memory.

    uint8_t* pJimgData    = (uint8_t*)jimgData;

    // Check if camera image data is not aligned.
    if( (uintptr_t)jimgData & 0xF )
    {
        // Allow for rescale if dimensions changed.
        if( w != (int)preProcessingState.alignedImgWidth ||
            h != (int)preProcessingState.alignedImgHeight )
        {
            if( preProcessingState.alignedImgBuf != NULL )
            {
                DPRINTF( "%s %d Creating aligned for preview\n",
                         __FILE__, __LINE__ );
                fcvMemFree( preProcessingState.alignedImgBuf );
                preProcessingState.alignedImgBuf = NULL;
            }
        }

        // Allocate buffer for aligned data if necessary.
        if( preProcessingState.alignedImgBuf == NULL )
        {
            preProcessingState.alignedImgWidth = w;
            preProcessingState.alignedImgHeight = h;
            preProcessingState.alignedImgBuf = (uint8_t*)fcvMemAlloc( w*h*3/2, 16 );
        }

        memcpy( preProcessingState.alignedImgBuf, jimgData, w*h*3/2 );
        pJimgData = preProcessingState.alignedImgBuf;
    }

    // Copy YUV image to render buffer
    memcpy(preProcessingState.preProcessImgBuf, pJimgData, w*h*3/2);

    // Allocate Memory to preprocess image
    if(preProcessingState.u16preProcessImgBuff == NULL)
    {
        preProcessingState.u16preProcessImgBuff = (uint16_t *)fcvMemAlloc(2*sizeof(uint8_t)*w*h, 16);
        if(preProcessingState.u16preProcessImgBuff == NULL)
        {
            EPRINTF("Allocate preprocess 16 bit image failed");
            return;
        }
    }

    if(preProcessingState.u32preProcessImgBuff == NULL)
    {
        preProcessingState.u32preProcessImgBuff = (uint32_t *)fcvMemAlloc(4*sizeof(uint8_t)*w*h, 16);
        if(preProcessingState.u32preProcessImgBuff == NULL)
        {
            EPRINTF("Allocate preprocess 32 bit image failed");
            return;
        }
    }
    if(preProcessingState.f32preProcessImgBuff == NULL)
    {
        preProcessingState.f32preProcessImgBuff = (float32_t *)fcvMemAlloc(4*sizeof(uint8_t)*w*h, 16);
        if(preProcessingState.f32preProcessImgBuff == NULL)
        {
            EPRINTF("Allocate preprocess 32 bit float image failed");
            return;
        }
    }
//    ////////////////////////////////////////////////////
//    if(preProcessingState.Offset == NULL)
//    {
//        preProcessingState.Offset = (float32_t *)fcvMemAlloc(4*sizeof(uint8_t)*w*h, 16);
//        if(preProcessingState.Offset == NULL)
//        {
//            EPRINTF("Allocate preprocess 16 bit image failed");
//            return;
//        }
//        else
//        {
//            fcvSetElementsf32(preProcessingState.Offset, w, h, 4*w, 205.43, 0, w);
//        }
//    }
//
//    if(preProcessingState.OffsetFFC == NULL)
//    {
//        preProcessingState.OffsetFFC = (float32_t *)fcvMemAlloc(4*sizeof(uint8_t)*w*h, 16);
//        if(preProcessingState.OffsetFFC == NULL)
//        {
//            EPRINTF("Allocate preprocess 16 bit image failed");
//            return;
//        }
//        else
//        {
//            fcvSetElementsf32(preProcessingState.OffsetFFC, w, h, 4*w, 31.09, 0, w);
//        }
//    }
//
//    if(preProcessingState.Gain_correction == NULL)
//    {
//        preProcessingState.Gain_correction = (float32_t *)fcvMemAlloc(4*sizeof(uint8_t)*w*h, 16);
//        if(preProcessingState.Gain_correction == NULL)
//        {
//            EPRINTF("Allocate preprocess 16 bit image failed");
//            return;
//        }
//        else
//        {
//            fcvSetElementsf32(preProcessingState.Gain_correction, w, h, 4*w, 1.2, 0, w);
//        }
//    }
//
//    if(preProcessingState.DeadPixel == NULL)
//    {
//        preProcessingState.DeadPixel = (uint8_t *)fcvMemAlloc(sizeof(uint8_t)*w*h, 16);
//        if(preProcessingState.DeadPixel == NULL)
//        {
//            EPRINTF("Allocate preprocess 16 bit image failed");
//            return;
//        }
//        else{
//            fcvSetElementsu8(preProcessingState.DeadPixel, w, h, w, 1, 0, w);
//        }
//    }

    // Process image here
    /** NUC */
    switch (preProcessingState.preProcessType)
    {
        case NO_PREPROCESS:
//            memcpy(preProcessingState.preProcessImgBuf, pJimgData, w*h);
            break;
        case NUC_PROCESS:
        {
            auto * ptr = (uint8_t *)fcvMemAlloc(w*h, 16);
            memset(ptr, 2, w*h);
            fcvElementMultiplyu8(preProcessingState.preProcessImgBuf, w, h, w, ptr, w, 2, FASTCV_CONVERT_POLICY_SATURATE, preProcessingState.preProcessImgBuf, w);
            memset(ptr, 50, w*h );
            fcvAddu8(preProcessingState.preProcessImgBuf, w, h, w, ptr, w, FASTCV_CONVERT_POLICY_SATURATE, preProcessingState.preProcessImgBuf, w);
            break;
        }
        case GAUSE:
        {
            for(int i = 0; i < h; i++)
            {
                for(int j = 0; j < w; j++)
                {
                    preProcessingState.f32preProcessImgBuff[i*w + j] = preProcessingState.preProcessImgBuf[i*w + j];
                }
            }

//            auto * ptr = (float32_t *)fcvMemAlloc(4*w*h, 16);
//            auto * ptr2 = (float32_t *)fcvMemAlloc(4*w*h, 16);
//
//            fcvSetElementsf32(ptr, w, h, 4*w, 200.5, 0, w);

//            DPRINTF( "%s %d ptr[0] value: %f\n",ptr[0],
//                     __FILE__, __LINE__ );
//            readDataFromTextFile(CONFIG_PATH_TINT, ptr, DATATYPE_FLOAT);

            fcvElementMultiplyf32(preProcessingState.f32preProcessImgBuff, w, h, w*4, preProcessingState.Gain_correction, w*4, preProcessingState.f32preProcessImgBuff, 4*w);
            fcvAddf32(preProcessingState.f32preProcessImgBuff, w, h, 4*w, preProcessingState.Offset, w*4,  preProcessingState.f32preProcessImgBuff, 4*w);

//            DPRINTF( "%s %d f32preProcessImgBuff[0] value: %f\n",preProcessingState.f32preProcessImgBuff[0],
//                     __FILE__, __LINE__ );

            for(int i = 0; i < h; i++)
            {
                for(int j = 0; j < w; j++)
                {
                    preProcessingState.preProcessImgBuf[i*w + j] = preProcessingState.f32preProcessImgBuff[i*w + j];
                }
            }
            break;
        }

        default:
            break;
    }

    /////////////////////
//    memcpy(preProcessingState.preProcessImgBuf + w*h, pJimgData + w*h, w*h/2);
    colorConvertYUV420ToRGB565Renderer(preProcessingState.preProcessImgBuf,
                                       w,
                                       h,
                                       (uint32_t*)renderBuffer );

    // Update image
    timeMs = ( util.getTimeMicroSeconds() - time ) / 1000.f;
    util.setProcessTime((util.getProcessTime()*(29.f/30.f))+(float)(timeMs/30.f));

    unlockRenderBuffer();
    // Let JNI know we don't need data anymore
    env->ReleaseByteArrayElements( img, jimgData, JNI_ABORT );
}
//---------------------------------------------------------------------------
/// @brief
///   Destroys the renderer
//---------------------------------------------------------------------------
extern "C" JNIEXPORT void JNICALL Java_com_qualcomm_fastcvdemo_apis_preprocessing_PreProcessing_cleanup
(
    JNIEnv * env,
    jobject obj
)
{
    if(preProcessingState.preProcessImgBuf != NULL)
    {
        fcvMemFree(preProcessingState.preProcessImgBuf);
        preProcessingState.preProcessImgBuf = NULL;
    }

    if(preProcessingState.alignedImgBuf != NULL)
    {
        fcvMemFree(preProcessingState.alignedImgBuf);
        preProcessingState.alignedImgBuf = NULL;
    }

    if(preProcessingState.f32preProcessImgBuff != NULL)
    {
        fcvMemFree(preProcessingState.f32preProcessImgBuff);
        preProcessingState.f32preProcessImgBuff = NULL;
    }

    if(preProcessingState.u16preProcessImgBuff != NULL)
    {
        fcvMemFree(preProcessingState.u16preProcessImgBuff);
        preProcessingState.u16preProcessImgBuff = NULL;
    }

    if(preProcessingState.u32preProcessImgBuff != NULL)
    {
        fcvMemFree(preProcessingState.u32preProcessImgBuff);
        preProcessingState.u32preProcessImgBuff = NULL;
    }

    if(preProcessingState.OffsetFFC != NULL)
    {
        fcvMemFree(preProcessingState.OffsetFFC);
        preProcessingState.OffsetFFC = NULL;
    }
    if(preProcessingState.Offset != NULL)
    {
        fcvMemFree(preProcessingState.Offset);
        preProcessingState.Offset = NULL;
    }
    if(preProcessingState.Gain_correction != NULL)
    {
        fcvMemFree(preProcessingState.Gain_correction);
        preProcessingState.Gain_correction = NULL;
    }
    if(preProcessingState.DeadPixel != NULL)
    {
        fcvMemFree(preProcessingState.DeadPixel);
        preProcessingState.DeadPixel = NULL;
    }

    fcvCleanUp();
}
extern "C" JNIEXPORT void JNICALL Java_com_qualcomm_fastcvdemo_apis_preprocessing_PreProcessing_init
(
    JNIEnv * env,
    jobject obj,
    jint w,
    jint h
)
{
    DPRINTF( "%s %d Initialize preprocess\n",
             __FILE__, __LINE__ );
    ////////////////////////////////////////////////////
    if(preProcessingState.Offset == NULL)
    {
        preProcessingState.Offset = (float32_t *)fcvMemAlloc(4*sizeof(uint8_t)*w*h, 16);
        if(preProcessingState.Offset == NULL)
        {
            EPRINTF("Allocate offset failed");
            return;
        }
        else
        {
            fcvSetElementsf32(preProcessingState.Offset, w, h, 4*w, 205.43, 0, w);
        }
    }

    if(preProcessingState.OffsetFFC == NULL)
    {
        preProcessingState.OffsetFFC = (float32_t *)fcvMemAlloc(4*sizeof(uint8_t)*w*h, 16);
        if(preProcessingState.OffsetFFC == NULL)
        {
            EPRINTF("Allocate offset FFC failed");
            return;
        }
        else
        {
            fcvSetElementsf32(preProcessingState.OffsetFFC, w, h, 4*w, 31.09, 0, w);
        }
    }

    if(preProcessingState.Gain_correction == NULL)
    {
        preProcessingState.Gain_correction = (float32_t *)fcvMemAlloc(4*sizeof(uint8_t)*w*h, 16);
        if(preProcessingState.Gain_correction == NULL)
        {
            EPRINTF("Allocate gain correction failed");
            return;
        }
        else
        {
            fcvSetElementsf32(preProcessingState.Gain_correction, w, h, 4*w, 1.2, 0, w);
        }
    }

    if(preProcessingState.DeadPixel == NULL)
    {
        preProcessingState.DeadPixel = (uint8_t *)fcvMemAlloc(sizeof(uint8_t)*w*h, 16);
        if(preProcessingState.DeadPixel == NULL)
        {
            EPRINTF("Allocate dead pixel failed");
            return;
        }
        else{
            fcvSetElementsu8(preProcessingState.DeadPixel, w, h, w, 1, 0, w);
        }
    }
}

