/**
 * \author Mr.Nobody
 * \file Fmac_Types.h
 * \ingroup Fmac
 * \brief Fmac module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file.
 *
 */

/**
 * \defgroup Fmac Fmac
 * \brief Filter math accelerator (FMAC) module
 */

#ifndef FMAC_FMAC_TYPES_H
#define FMAC_FMAC_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpdma_Port.h"                     /* DMA types used by fmac_Config_t */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define FMAC_NULL_PTR                       ( ( void* ) 0u )

/** Size of the FMAC local memory in 16-bit words (coefficients, input and output buffers share it) */
#define FMAC_MEMORY_WORDS                   ( 256u )

/** Minimal count of the coefficients of a FIR filter, a convolution, a correlation and a dot product */
#define FMAC_FIR_COEF_MIN                   ( 2u )

/** Maximal count of the coefficients of a FIR filter, a convolution, a correlation and a dot product */
#define FMAC_FIR_COEF_MAX                   ( 127u )

/** Minimal count of the feedforward coefficients (B) of an IIR filter */
#define FMAC_IIR_FEEDFORWARD_MIN            ( 2u )

/** Maximal count of the feedforward coefficients (B) of an IIR filter */
#define FMAC_IIR_FEEDFORWARD_MAX            ( 64u )

/** Minimal count of the feedback coefficients (A) of an IIR filter */
#define FMAC_IIR_FEEDBACK_MIN               ( 1u )

/** Maximal count of the feedback coefficients (A) of an IIR filter */
#define FMAC_IIR_FEEDBACK_MAX               ( 63u )

/** Maximal count of the samples of one request in \ref FMAC_XFER_MODE_DMA (a Gpdma block has 65535 bytes at most) */
#define FMAC_DMA_SAMPLE_CNT_MAX             ( 32766u )

/* ========================== EXPORTED MACROS =============================== */

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t fmac_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t fmac_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t fmac_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    fmac_MajorVersion_t Major; /**< Major version */
    fmac_MinorVersion_t Minor; /**< Minor version */
    fmac_PatchVersion_t Patch; /**< Patch version */
}   fmac_ModuleVersion_t;


/** \brief Function status enumeration */
typedef enum
{
    FMAC_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    FMAC_FUNCTION_ACTIVE         /**< Function status is active   */
}   fmac_FunctionState_t;


/** \brief Flag states enumeration */
typedef enum
{
    FMAC_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    FMAC_FLAG_ACTIVE         /**< Active flag state   */
}   fmac_FlagState_t;


/** \brief Enumeration used to signal request processing state */
typedef enum
{
    FMAC_REQUEST_ERROR = 0u, /**< Processing request failed  */
    FMAC_REQUEST_OK          /**< Processing request succeed */
}   fmac_RequestState_t;


/** \brief Interrupt priority type definition */
typedef uint32_t fmac_IrqPrio_t;


/** \brief One sample or one coefficient: a signed 16-bit number in the q1.15 fixed-point format
 *         (-1.0 is 0x8000, the largest value 0x7FFF is 1 - 2^-15) */
typedef int16_t fmac_Sample_t;


/** \brief Count of the samples or of the coefficients, index into such an array, size in the 16-bit words of the
 *         local memory */
typedef uint32_t fmac_SampleCnt_t;


/** \brief Function executed by the accelerator.
 *
 * FIR and IIR are streaming filters: the state (the history of the input and, for the IIR, of the output) is kept
 * between the requests. The vector functions are independent calculations: every request starts from the zero state
 * and is processed by the FIR function of the hardware. With the coefficients B[ k ] (k = 0 .. N-1), the feedback
 * coefficients A[ k ] (k = 1 .. M, stored from the index 0), the input x and the output y (zero for a negative index):
 * - \ref FMAC_FUNC_FIR -         y[n] = 2^gain * sum( B[k] * x[n-k] ), the state is kept between the requests
 * - \ref FMAC_FUNC_IIR -         y[n] = 2^gain * ( sum( B[k] * x[n-k] ) + sum( A[k] * y[n-k] ) ), the state is kept
 * - \ref FMAC_FUNC_CONVOLUTION - y[n] = 2^gain * sum( B[k] * x[n-k] ), zero state in every request
 * - \ref FMAC_FUNC_CORRELATION - y[n] = 2^gain * sum( B[k] * x[n-N+1+k] ), zero state in every request
 * - \ref FMAC_FUNC_DOT_PRODUCT - y = 2^gain * sum( B[k] * x[k] ), exactly N inputs and one result
 */
typedef enum
{
    FMAC_FUNC_FIR = 0u,    /**< Finite impulse response filter                                 */
    FMAC_FUNC_IIR,         /**< Infinite impulse response filter (direct form 1)               */
    FMAC_FUNC_DOT_PRODUCT, /**< Dot product of the coefficient vector and a vector of N inputs */
    FMAC_FUNC_CONVOLUTION, /**< Convolution of the coefficient vector with an input vector     */
    FMAC_FUNC_CORRELATION, /**< Correlation of an input vector with the coefficient vector     */
    FMAC_FUNC_CNT          /**< Count of available options                                     */
}   fmac_Function_t;


/** \brief Gain applied to the output of the accumulator (the result is multiplied by 2^R, the value is R) */
typedef enum
{
    FMAC_GAIN_1   = 0u, /**< Gain 1   (0 dB)            */
    FMAC_GAIN_2   = 1u, /**< Gain 2   (6 dB)            */
    FMAC_GAIN_4   = 2u, /**< Gain 4   (12 dB)           */
    FMAC_GAIN_8   = 3u, /**< Gain 8   (18 dB)           */
    FMAC_GAIN_16  = 4u, /**< Gain 16  (24 dB)           */
    FMAC_GAIN_32  = 5u, /**< Gain 32  (30 dB)           */
    FMAC_GAIN_64  = 6u, /**< Gain 64  (36 dB)           */
    FMAC_GAIN_128 = 7u, /**< Gain 128 (42 dB)           */
    FMAC_GAIN_CNT       /**< Count of available options */
}   fmac_Gain_t;


/** \brief Handling of a result which exceeds the q1.15 range after the gain */
typedef enum
{
    FMAC_CLIP_SATURATE = 0u, /**< The result is saturated to the largest positive or negative value */
    FMAC_CLIP_NONE,          /**< The result is not saturated (the unused bits are truncated)       */
    FMAC_CLIP_CNT            /**< Count of available options                                        */
}   fmac_Clipping_t;


/** \brief Watermark of a circular buffer: the count of the samples transferred under one request (interrupt).
 *
 * The watermark has to be \ref FMAC_WATERMARK_1 for a buffer served by the DMA.
 */
typedef enum
{
    FMAC_WATERMARK_1 = 0u, /**< Threshold of 1 sample      */
    FMAC_WATERMARK_2,      /**< Threshold of 2 samples     */
    FMAC_WATERMARK_4,      /**< Threshold of 4 samples     */
    FMAC_WATERMARK_8,      /**< Threshold of 8 samples     */
    FMAC_WATERMARK_CNT     /**< Count of available options */
}   fmac_Watermark_t;


/** \brief Data transfer handling style enumeration */
typedef enum
{
    FMAC_XFER_MODE_POLL = 0u, /**< Samples are moved by \ref Fmac_Task                                    */
    FMAC_XFER_MODE_ISR,       /**< Samples are moved by the FMAC interrupt                                */
    FMAC_XFER_MODE_DMA,       /**< Samples are moved by two Gpdma channels, the FMAC interrupt reports errors */
    FMAC_XFER_MODE_CNT        /**< Count of available options                                             */
}   fmac_XferMode_t;


/** \brief Mode of the buffers of a stream */
typedef enum
{
    FMAC_BUFFER_CIRCULAR = 0u, /**< The stream restarts at the begin of the buffers until it is stopped */
    FMAC_BUFFER_ONE_SHOT,      /**< The stream ends after one pass through the buffers                  */
    FMAC_BUFFER_CNT            /**< Count of available options                                          */
}   fmac_BufferMode_t;


/** \brief State of the filter */
typedef enum
{
    FMAC_FILTER_UNCONFIGURED = 0u, /**< No filter is configured                                           */
    FMAC_FILTER_READY,             /**< The filter is configured and preloaded, it can be started         */
    FMAC_FILTER_RUNNING,           /**< The filter is started and accepts the requests                    */
    FMAC_FILTER_STOPPED,           /**< The filter was stopped, a preload is required before the restart  */
    FMAC_FILTER_CNT                /**< Count of available options                                        */
}   fmac_FilterState_t;


/** \brief State of the processing of the data */
typedef enum
{
    FMAC_XFER_STATE_IDLE = 0u, /**< No request is being processed                                  */
    FMAC_XFER_STATE_BUSY,      /**< A block request or a stream is being processed                 */
    FMAC_XFER_STATE_ERROR,     /**< The last request ended with an error, see \ref Fmac_Get_XferError */
    FMAC_XFER_STATE_CNT        /**< Count of available options                                     */
}   fmac_XferState_t;


/** \brief List of the errors of the processing (bit mask) */
typedef enum
{
    FMAC_XFER_ERROR_NONE       = 0x00u, /**< No error                                                   */
    FMAC_XFER_ERROR_SATURATION = 0x01u, /**< The accumulator exceeded its range, the processing goes on */
    FMAC_XFER_ERROR_UNDERFLOW  = 0x02u, /**< A result was read from an empty output buffer              */
    FMAC_XFER_ERROR_OVERFLOW   = 0x04u, /**< A sample was written to a full input buffer                */
    FMAC_XFER_ERROR_DMA        = 0x08u, /**< A Gpdma transfer error                                     */
    FMAC_XFER_ERROR_ALL        = 0x0Fu  /**< All possible errors                                        */
}   fmac_XferError_t;


/** \brief Callback of the half of the output buffer (a block request or a stream), called from the interrupt context
 *         or from \ref Fmac_Task */
typedef void ( fmac_XferHalfCallback_t )( void );


/** \brief Callback of the end of the output buffer (a block request or a lap of a stream), called from the interrupt
 *         context or from \ref Fmac_Task */
typedef void ( fmac_XferCompleteCallback_t )( void );


/** \brief Callback of an error, called from the interrupt context or from \ref Fmac_Task
 *
 * \param errorMask [in]: The errors detected since the last call, bit mask of \ref fmac_XferError_t
 */
typedef void ( fmac_ErrorCallback_t )( fmac_XferError_t errorMask );


/** \brief Type used to store Fmac module configuration (see \ref Fmac_Init) */
typedef struct
{
    fmac_XferMode_t               XferMode;             /**< Data transfer mode (poll / interrupt / DMA)           */
    gpdma_PeriphId_t              DmaPeriphId;          /**< Gpdma controller (DMA mode only)                      */
    gpdma_ChannelId_t             DmaWriteChannelId;    /**< Gpdma channel of the input data (DMA mode only)       */
    gpdma_ChannelId_t             DmaReadChannelId;     /**< Gpdma channel of the results (DMA mode only)          */
    gpdma_Priority_t              DmaPriority;          /**< Priority of both Gpdma channels (DMA mode only)       */
    fmac_IrqPrio_t                IrqPriority;          /**< FMAC interrupt priority (interrupt and DMA mode)      */
    fmac_XferHalfCallback_t     * XferHalfCallback;     /**< Half of the output buffer, \ref FMAC_NULL_PTR disables  */
    fmac_XferCompleteCallback_t * XferCompleteCallback; /**< End of the output buffer, \ref FMAC_NULL_PTR disables   */
    fmac_ErrorCallback_t        * ErrorCallback;        /**< Error of the processing, \ref FMAC_NULL_PTR disables    */
}   fmac_Config_t;


/** \brief Filter configuration (see \ref Fmac_Set_FilterConfig).
 *
 * The module lays out the local memory as the coefficients (X2), the input buffer (X1) and the output buffer (Y).
 * The input buffer holds as many samples as the feedforward coefficients plus the input headroom, the output buffer
 * as many samples as the feedback coefficients (IIR) plus the output headroom. The headroom is the larger one of the
 * requested count and of the watermark threshold of the buffer. Every member has a usable zero value except the
 * function, the coefficients and their counts.
 */
typedef struct
{
    fmac_Function_t       Function;        /**< Function of the accelerator                                        */
    const fmac_Sample_t * B;               /**< Feedforward coefficients, the vector of a vector function          */
    fmac_SampleCnt_t      BCount;          /**< Count of the coefficients \ref B, see \ref FMAC_FIR_COEF_MIN       */
    const fmac_Sample_t * A;               /**< Feedback coefficients of an IIR (else ignored)                     */
    fmac_SampleCnt_t      ACount;          /**< Count of the coefficients \ref A (IIR only, else ignored)          */
    fmac_Gain_t           Gain;            /**< Gain of the output of the accumulator                              */
    fmac_Clipping_t       Clipping;        /**< Handling of a result out of the q1.15 range                        */
    fmac_Watermark_t      InputWatermark;  /**< Watermark of the input buffer                                      */
    fmac_Watermark_t      OutputWatermark; /**< Watermark of the output buffer                                     */
    fmac_SampleCnt_t      InputHeadroom;   /**< Spare samples of the input buffer, 0 for the minimum               */
    fmac_SampleCnt_t      OutputHeadroom;  /**< Spare samples of the output buffer, 0 for the minimum              */
}   fmac_FilterConfig_t;


/** \brief Request of a stream (see \ref Fmac_Set_StreamStart).
 *
 * The input and the output buffer have the same size (one result per sample). The half callback is called when half
 * of the output buffer is filled, the complete callback when it is full (the stream starts again at the begin of the
 * buffers in \ref FMAC_BUFFER_CIRCULAR mode).
 */
typedef struct
{
    const fmac_Sample_t * Input;      /**< Input samples, filled by the application or by another DMA channel  */
    fmac_SampleCnt_t      InputSize;  /**< Size of the input buffer in samples                                 */
    fmac_Sample_t       * Output;     /**< Buffer of the results                                               */
    fmac_SampleCnt_t      OutputSize; /**< Size of the output buffer in samples, equal to \ref InputSize       */
    fmac_BufferMode_t     BufferMode; /**< Circular or one shot buffers                                        */
}   fmac_StreamRequest_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

#ifdef __cplusplus
}
#endif

#endif /* FMAC_FMAC_TYPES_H */
