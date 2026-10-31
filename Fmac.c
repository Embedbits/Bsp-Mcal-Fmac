/**
 * \author Mr.Nobody
 * \file Fmac.c
 * \ingroup Fmac
 * \brief Fmac module common functionality
 *
 * # Introduction #
 * The filter math accelerator (FMAC) executes the FIR and the IIR filters (and the vector functions) on 16-bit
 * fixed-point (q1.15) data with a 256-word local memory. The module lays out the local memory from the filter
 * configuration (the coefficients X2, the input buffer X1 and the output buffer Y, RM0481 / RM0517 FMAC chapter),
 * loads the coefficients and the initial state (preload, zero by default), starts the filter and moves the samples.
 *
 * # Device differences (STM32H5) #
 * The FMAC exists on STM32H562, H563, H573 and on the STM32H5E / H5F lines (CMSIS device headers). On the other
 * lines every public function checks the CMSIS instance macro and returns \ref FMAC_REQUEST_ERROR.
 *
 * # Memory layout #
 * With N feedforward coefficients, M feedback coefficients (IIR only) and the headroom d1 of the input buffer and d2
 * of the output buffer (the larger one of the requested headroom and of the watermark threshold of the buffer):
 * X2 = N + M words at 0, X1 = N + d1 words behind X2, Y = M + d2 words behind X1, so 2N + 2M + d1 + d2 words in all.
 * The areas do not overlap (the recommendation of the reference manual), the total must not exceed 256 words.
 *
 * # State of the filter #
 * \ref Fmac_Set_FilterConfig resets the unit, programs the buffers and loads the coefficients and a zero state (N - 1
 * zero input samples, M zero output samples), so the first input sample gives the first output sample.
 * \ref Fmac_Set_FilterPreload loads another history. \ref Fmac_Set_FilterStart starts the hardware function (FIR or
 * IIR), \ref Fmac_Set_FilterStop stops it, the state of the pipeline is then undefined and a preload is required.
 *
 * # Functions #
 * FIR and IIR are streaming filters: the state is kept between the requests. The vector functions are independent
 * calculations on the FIR function of the hardware, every request resets the unit, loads the coefficients and the
 * zero state and starts and stops the hardware function: the convolution (one result per input sample, zero state),
 * the correlation (the convolution with the coefficients in the reverse order) and the dot product (the N inputs are
 * preloaded into X1, the first result is the dot product, with the coefficients in the reverse order).
 *
 * # Transfer modes #
 * A request (\ref Fmac_Set_FilterProcess, \ref Fmac_Set_StreamStart) is processed in the background:
 * - \ref FMAC_XFER_MODE_POLL - by \ref Fmac_Task,
 * - \ref FMAC_XFER_MODE_ISR - by the FMAC interrupt (the write interrupt serves the input buffer, the read interrupt
 *   the output buffer),
 * - \ref FMAC_XFER_MODE_DMA - by two Gpdma channels (the input samples to FMAC_WDATA, the results from FMAC_RDATA, one
 *   16-bit sample per request), the FMAC interrupt reports the errors. The Gpdma channels are initialized by
 *   \ref Fmac_Init and armed by every request (a cyclic transfer list node for a circular stream); the transfer
 *   complete and the half transfer interrupts of the channel of the results call the callbacks.
 * In the poll and the interrupt mode the flags FMAC_SR.X1FULL and FMAC_SR.YEMPTY decide how many samples are moved.
 * A stream runs the same way on a circular input and output buffer of the application: the half callback is called
 * when half of the output buffer is filled, the complete callback at the end of every lap. The overflow and the
 * underflow of the buffers (and a Gpdma error) are reported by the error callback and end the request, the saturation
 * of the accumulator is reported and the request goes on.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Fmac.h"                           /* Self include                   */
#include "Fmac_Port.h"                      /* Own port file include          */
#include "Fmac_Types.h"                     /* Module types definitions       */
#include "Fmac_Reg.h"                       /* Register access                */
#include "Nvic_Port.h"                      /* NVIC handler functionality     */
#include "Rcc_Port.h"                       /* RCC handler functionality      */
#include "Gpdma_Port.h"                     /* DMA handler functionality      */
#include "Stm32.h"                          /* FMAC registers definition      */
/* ============================== TYPEDEFS ================================== */

#if defined(FMAC)

/** Size of the copy of the coefficients: the largest content of the X2 buffer (FIR 127, IIR 64 + 63 coefficients),
 *  needed ahead of \ref fmac_RuntimeData_t */
#define FMAC_COEF_IMAGE_SIZE             ( FMAC_FIR_COEF_MAX )

_Static_assert( ( FMAC_IIR_FEEDFORWARD_MAX + FMAC_IIR_FEEDBACK_MAX ) <= FMAC_COEF_IMAGE_SIZE,
                "Fmac: the coefficient image does not fit the IIR filter." );

/** \brief Gpdma channels of the DMA mode */
typedef enum
{
    FMAC_DMA_STREAM_WRITE = 0u, /**< Channel of the input samples (memory to FMAC_WDATA) */
    FMAC_DMA_STREAM_READ,       /**< Channel of the results (FMAC_RDATA to memory)        */
    FMAC_DMA_STREAM_CNT         /**< Count of the channels                                */
}   fmac_DmaStream_t;

/** \brief Runtime data of the module */
typedef struct
{
    fmac_FunctionState_t         InitState;            /**< Fmac_Init succeeded                                     */
    fmac_XferMode_t              XferMode;             /**< Configured data transfer mode                           */
    gpdma_PeriphId_t             DmaPeriphId;          /**< Gpdma controller (DMA mode only)                        */
    gpdma_ChannelId_t            DmaWriteChannelId;    /**< Gpdma channel of the input data (DMA mode only)         */
    gpdma_ChannelId_t            DmaReadChannelId;     /**< Gpdma channel of the results (DMA mode only)            */
    gpdma_Priority_t             DmaPriority;          /**< Priority of the Gpdma channels (DMA mode only)          */
    fmac_XferHalfCallback_t     *XferHalfCallback;     /**< Half of the output buffer callback                      */
    fmac_XferCompleteCallback_t *XferCompleteCallback; /**< End of the output buffer callback                       */
    fmac_ErrorCallback_t        *ErrorCallback;        /**< Error callback                                          */
    fmac_FilterState_t           FilterState;          /**< State of the filter                                     */
    fmac_FilterConfig_t          Filter;               /**< Copy of the filter configuration (without pointers)     */
    fmac_MemoryLayout_t          Layout;               /**< Layout of the local memory of the filter                */
    fmac_Sample_t                CoefImage[ FMAC_COEF_IMAGE_SIZE ]; /**< Content of the X2 buffer, as loaded     */
    fmac_XferState_t             XferState;            /**< State of the processing of the data                     */
    fmac_XferError_t             XferError;            /**< Errors latched since the start of the request           */
    fmac_XferError_t             ErrorBaseline;        /**< Errors of the hardware flags already set at the start   */
    const fmac_Sample_t         *ReqInput;             /**< Input samples of the request                            */
    fmac_Sample_t               *ReqOutput;            /**< Buffer of the results of the request                    */
    fmac_SampleCnt_t             ReqInputCnt;          /**< Count of the input samples of the request               */
    fmac_SampleCnt_t             ReqOutputCnt;         /**< Count of the results of the request                     */
    fmac_SampleCnt_t             ReqInputIdx;          /**< Count of the input samples written to the filter        */
    fmac_SampleCnt_t             ReqOutputIdx;         /**< Count of the results read from the filter               */
    fmac_SampleCnt_t             ReqHalfIdx;           /**< Count of the results at which the half callback is called */
    fmac_FunctionState_t         ReqStream;            /**< The request is a stream                                 */
    fmac_BufferMode_t            ReqBufferMode;        /**< Buffer mode of the stream                               */
    fmac_FunctionState_t         DmaChannelInit[ FMAC_DMA_STREAM_CNT ]; /**< Gpdma channel initialized by the module */
}   fmac_RuntimeData_t;

#endif /* FMAC */

/* ======================== FORWARD DECLARATIONS ============================ */

#if defined(FMAC)

static fmac_RequestState_t Fmac_Check_Config       ( const fmac_Config_t * const fmacConfig );
static fmac_RequestState_t Fmac_Check_FilterConfig ( const fmac_FilterConfig_t * const filterConfig );
static fmac_RequestState_t Fmac_Check_Block        ( const fmac_Sample_t * const input,
                                                     fmac_SampleCnt_t inputCnt,
                                                     fmac_Sample_t * const output,
                                                     fmac_SampleCnt_t outputCnt,
                                                     fmac_SampleCnt_t * const feedCnt,
                                                     fmac_SampleCnt_t * const resultCnt );
static fmac_RequestState_t Fmac_Check_Stream       ( const fmac_StreamRequest_t * const streamRequest );
static fmac_FunctionState_t Fmac_Is_VectorFunction ( void );
static fmac_FunctionState_t Fmac_Is_Circular       ( void );
static fmac_RequestState_t Fmac_Check_History      ( const fmac_Sample_t * const history,
                                                     fmac_SampleCnt_t historyCnt,
                                                     fmac_SampleCnt_t maxCnt );
static void                Fmac_Calc_Layout        ( const fmac_FilterConfig_t * const filterConfig,
                                                     fmac_MemoryLayout_t * const layout );
static fmac_SampleCnt_t    Fmac_Get_Headroom       ( fmac_SampleCnt_t requestedCnt, fmac_Watermark_t watermark );
static void                Fmac_Build_CoefImage    ( const fmac_FilterConfig_t * const filterConfig );
static fmac_RequestState_t Fmac_Program_Filter     ( void );
static fmac_RequestState_t Fmac_Load_State         ( const fmac_Sample_t * const inputHistory,
                                                     fmac_SampleCnt_t inputCnt,
                                                     const fmac_Sample_t * const outputHistory,
                                                     fmac_SampleCnt_t outputCnt );
static fmac_RequestState_t Fmac_Load_Buffer        ( const fmac_RegFuncParam_t * const funcParam,
                                                     fmac_SampleCnt_t totalCnt,
                                                     const fmac_Sample_t * const values,
                                                     fmac_SampleCnt_t valueCnt );
static fmac_RequestState_t Fmac_Start_Filter       ( void );
static fmac_RequestState_t Fmac_Run_Function       ( void );
static void                Fmac_Enable_ErrorIrqs   ( void );
static void                Fmac_Set_Request        ( const fmac_Sample_t * const input,
                                                     fmac_SampleCnt_t feedCnt,
                                                     fmac_Sample_t * const output,
                                                     fmac_SampleCnt_t resultCnt,
                                                     fmac_FunctionState_t stream,
                                                     fmac_BufferMode_t bufferMode );
static fmac_RequestState_t Fmac_Start_Request      ( void );
static fmac_RequestState_t Fmac_Arm_Vector         ( void );
static fmac_RequestState_t Fmac_Stop_Transfer      ( void );
static fmac_RequestState_t Fmac_Init_Hardware      ( fmac_IrqPrio_t irqPrio );
static fmac_RequestState_t Fmac_Release            ( void );
static void                Fmac_Reset_Runtime      ( void );

static void                Fmac_Service            ( void );
static void                Fmac_Feed               ( void );
static void                Fmac_Drain              ( void );
static void                Fmac_Check_Errors       ( void );
static void                Fmac_Report_Errors      ( fmac_XferError_t newErrors );
static void                Fmac_Finish_Request     ( void );
static void                Fmac_Abort_Request      ( void );
static fmac_XferError_t    Fmac_Get_HwErrors       ( void );

static void                Fmac_IsrHandler         ( void );

static fmac_RequestState_t Fmac_Dma_Init           ( void );
static fmac_RequestState_t Fmac_Dma_Deinit         ( void );
static fmac_RequestState_t Fmac_Dma_Stop           ( void );
static fmac_RequestState_t Fmac_Dma_Start          ( void );
static fmac_RequestState_t Fmac_Dma_Arm            ( fmac_DmaStream_t stream,
                                                     uint32_t memoryAddr,
                                                     fmac_SampleCnt_t sampleCnt );
static void                Fmac_Dma_Get_XferConfig ( fmac_DmaStream_t stream,
                                                     uint32_t memoryAddr,
                                                     fmac_SampleCnt_t sampleCnt,
                                                     fmac_BufferMode_t bufferMode,
                                                     gpdma_TransferConfig_t * const xferConfig );
static gpdma_ChannelId_t   Fmac_Dma_Get_ChannelId  ( fmac_DmaStream_t stream );
static void                Fmac_Dma_ReadComplete   ( void );
static void                Fmac_Dma_ReadHalf       ( void );
static void                Fmac_Dma_Error          ( gpdma_ErrorMaskId_t errorMask );

#endif /* FMAC */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define FMAC_MAJOR_VERSION               ( 0u )

/** Value of minor version of SW module */
#define FMAC_MINOR_VERSION               ( 1u )

/** Value of patch version of SW module */
#define FMAC_PATCH_VERSION               ( 0u )

/** Default interrupt priority used by \ref Fmac_Get_DefaultConfig */
#define FMAC_DEFAULT_IRQ_PRIO            ( 0u )

/** Register value without any bit set */
#define FMAC_BITS_CLEAR                  ( 0u )

/** Count of the history samples of the input buffer lost by the first output sample (N - 1 for N coefficients) */
#define FMAC_HISTORY_OFFSET              ( 1u )

/** Base of the watermark threshold: the threshold is this value shifted left by the watermark */
#define FMAC_WATERMARK_UNIT              ( 1u )

/** Count of the results of a dot product */
#define FMAC_DOT_RESULT_CNT              ( 1u )

/** Smallest size of the buffers of a stream (the half callback needs two samples) */
#define FMAC_STREAM_SIZE_MIN             ( 2u )

/** Count of the transfers of a Gpdma channel (one transfer list node) */
#define FMAC_DMA_TRANSFERS_CNT           ( 1u )

/** Count of the data items of a Gpdma burst (one request, one sample) */
#define FMAC_DMA_BURST_LEN               ( 1u )

/** Count of the samples of the transfer configured at the initialization of a Gpdma channel (placeholder) */
#define FMAC_DMA_PLACEHOLDER_CNT         ( 1u )

/** Gpdma errors which end the request */
#define FMAC_DMA_ERROR_MASK              ( GPDMA_ERROR_TRANSFER      | \
                                           GPDMA_ERROR_CONFIG_UPDATE | \
                                           GPDMA_ERROR_CONFIG_ERROR  | \
                                           GPDMA_ERROR_TRIG_OVERRUN    )

/** Divisor giving the half of a count of samples */
#define FMAC_HALF_DIVISOR                ( 2u )

/** Scale of the q1.15 format: the value of 1.0 */
#define FMAC_Q15_SCALE                   ( 32768.0f )

/** Rounding offset of the conversion to the q1.15 format */
#define FMAC_Q15_HALF                    ( 0.5f )

/** Zero of the floating point conversion */
#define FMAC_Q15_ZERO                    ( 0.0f )

/** Scaled value from which the conversion saturates to the largest q1.15 sample */
#define FMAC_Q15_MAX_FLOAT               ( 32767.0f )

/** Scaled value up to which the conversion saturates to the smallest q1.15 sample */
#define FMAC_Q15_MIN_FLOAT               ( -32768.0f )

/** Largest q1.15 sample */
#define FMAC_Q15_MAX                     ( (fmac_Sample_t)INT16_MAX )

/** Smallest q1.15 sample */
#define FMAC_Q15_MIN                     ( (fmac_Sample_t)INT16_MIN )

/** Hardware error flags of FMAC_SR reported by the module */
#define FMAC_SR_ERROR_MASK               ( FMAC_SR_SAT | FMAC_SR_UNFL | FMAC_SR_OVFL )

/** Errors which end the request (the data of the request are lost, the pipeline of the filter is undefined) */
#define FMAC_XFER_ERROR_FATAL            ( FMAC_XFER_ERROR_UNDERFLOW | \
                                           FMAC_XFER_ERROR_OVERFLOW  | \
                                           FMAC_XFER_ERROR_DMA         )

/** Interrupt enable bits of the error flags of FMAC_CR */
#define FMAC_CR_ERROR_IRQ_MASK           ( FMAC_CR_SATIEN | FMAC_CR_UNFLIEN | FMAC_CR_OVFLIEN )

/** Interrupt enable bits of the data requests of FMAC_CR */
#define FMAC_CR_DATA_IRQ_MASK            ( FMAC_CR_WIEN | FMAC_CR_RIEN )

/** Enable bits of the data requests of FMAC_CR (interrupts and DMA) */
#define FMAC_CR_DATA_REQ_MASK            ( FMAC_CR_WIEN   | FMAC_CR_RIEN   | \
                                           FMAC_CR_DMAWEN | FMAC_CR_DMAREN   )

/** Bits of FMAC_CR which belong to the running filter (error interrupts, data requests), cleared when it stops */
#define FMAC_CR_RUN_MASK                 ( FMAC_CR_ERROR_IRQ_MASK | FMAC_CR_DATA_REQ_MASK )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

#if defined(FMAC)

/** \brief Runtime data of the module */
static fmac_RuntimeData_t fmac_RuntimeData =
{
    .InitState            = FMAC_FUNCTION_INACTIVE,
    .XferMode             = FMAC_XFER_MODE_POLL,
    .DmaPeriphId          = GPDMA_PERIPH_1,
    .DmaWriteChannelId    = GPDMA_CHANNEL_0,
    .DmaReadChannelId     = GPDMA_CHANNEL_1,
    .DmaPriority          = GPDMA_PRIORITY_LOW,
    .XferHalfCallback     = FMAC_NULL_PTR,
    .XferCompleteCallback = FMAC_NULL_PTR,
    .ErrorCallback        = FMAC_NULL_PTR,
    .FilterState          = FMAC_FILTER_UNCONFIGURED,
    .XferState            = FMAC_XFER_STATE_IDLE,
    .XferError            = FMAC_XFER_ERROR_NONE,
    .ErrorBaseline        = FMAC_XFER_ERROR_NONE,
    .ReqStream            = FMAC_FUNCTION_INACTIVE,
    .ReqBufferMode        = FMAC_BUFFER_ONE_SHOT,
};

/** \brief Gpdma request of the channels */
static const gpdma_PeriphReqId_t fmac_DmaRequestLut[ FMAC_DMA_STREAM_CNT ] =
{
    [FMAC_DMA_STREAM_WRITE] = GPDMA_REQ_FMAC_WRITE,
    [FMAC_DMA_STREAM_READ]  = GPDMA_REQ_FMAC_READ,
};

/** \brief Direction of the transfer of the channels */
static const gpdma_Direction_t fmac_DmaDirectionLut[ FMAC_DMA_STREAM_CNT ] =
{
    [FMAC_DMA_STREAM_WRITE] = GPDMA_DIR_MEMORY_TO_PERIPH,
    [FMAC_DMA_STREAM_READ]  = GPDMA_DIR_PERIPH_TO_MEMORY,
};

/** \brief Gpdma channel type (the channels 6 and 7 support the 2D addressing): layout of the transfer list node */
static const gpdma_ChannelType_t fmac_DmaChannelTypeLut[ GPDMA_CHANNEL_CNT ] =
{
    [GPDMA_CHANNEL_0] = GPDMA_CHANNEL_LINEAR,
    [GPDMA_CHANNEL_1] = GPDMA_CHANNEL_LINEAR,
    [GPDMA_CHANNEL_2] = GPDMA_CHANNEL_LINEAR,
    [GPDMA_CHANNEL_3] = GPDMA_CHANNEL_LINEAR,
    [GPDMA_CHANNEL_4] = GPDMA_CHANNEL_LINEAR,
    [GPDMA_CHANNEL_5] = GPDMA_CHANNEL_LINEAR,
    [GPDMA_CHANNEL_6] = GPDMA_CHANNEL_LINEAR_2D,
    [GPDMA_CHANNEL_7] = GPDMA_CHANNEL_LINEAR_2D,
};

/** \brief Gpdma transfer list nodes of the channels (must be static, used by the Gpdma hardware) */
static gpdma_XferList_t fmac_DmaXferList[ FMAC_DMA_STREAM_CNT ];

/** \brief Memory of the placeholder transfer configured at the initialization of the Gpdma channels */
static fmac_Sample_t fmac_DmaDummy;

#endif /* FMAC */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
fmac_ModuleVersion_t Fmac_Get_ModuleVersion( void )
{
    fmac_ModuleVersion_t retVersion;

    retVersion.Major = FMAC_MAJOR_VERSION;
    retVersion.Minor = FMAC_MINOR_VERSION;
    retVersion.Patch = FMAC_PATCH_VERSION;

    return ( retVersion );
}


/**
 * \brief Fills configuration structure with default values.
 *
 * Default configuration: POLL transfer mode, Gpdma1 channel 0 for the input data and channel 1 for the results at
 * low priority (DMA mode only), interrupt priority 0, no callback.
 *
 * \param fmacConfig [out]: Pointer to store default configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Get_DefaultConfig( fmac_Config_t * const fmacConfig )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( FMAC_NULL_PTR != fmacConfig )
    {
        fmacConfig->XferMode             = FMAC_XFER_MODE_POLL;
        fmacConfig->DmaPeriphId          = GPDMA_PERIPH_1;
        fmacConfig->DmaWriteChannelId    = GPDMA_CHANNEL_0;
        fmacConfig->DmaReadChannelId     = GPDMA_CHANNEL_1;
        fmacConfig->DmaPriority          = GPDMA_PRIORITY_LOW;
        fmacConfig->IrqPriority          = FMAC_DEFAULT_IRQ_PRIO;
        fmacConfig->XferHalfCallback     = FMAC_NULL_PTR;
        fmacConfig->XferCompleteCallback = FMAC_NULL_PTR;
        fmacConfig->ErrorCallback        = FMAC_NULL_PTR;

        retState = FMAC_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, nothing is filled */
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)fmacConfig;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Initializes the Fmac module.
 *
 * Enables the peripheral clock (Rcc), resets the FMAC unit, clears the control register and, for
 * \ref FMAC_XFER_MODE_ISR and \ref FMAC_XFER_MODE_DMA, configures the FMAC interrupt (Nvic); in DMA mode the two
 * Gpdma channels are initialized too. The result is verified by a read-back of the registers, on a failure the module
 * is deinitialized and \ref FMAC_REQUEST_ERROR is returned. An initialized module has to be deinitialized by
 * \ref Fmac_Deinit before the next initialization.
 *
 * \pre   \p fmacConfig shall not be \ref FMAC_NULL_PTR, \ref fmac_Config_t::XferMode shall be a valid value and, in
 *        DMA mode, the Gpdma controller, the two different channels and the priority shall be valid; otherwise
 *        \ref FMAC_REQUEST_ERROR and nothing changes.
 *
 * \param fmacConfig [in]: Pointer to Fmac configuration, see \ref fmac_Config_t. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Init( const fmac_Config_t * const fmacConfig )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    const fmac_RequestState_t configState = Fmac_Check_Config( fmacConfig );

    if( ( FMAC_REQUEST_OK        == configState                ) &&
        ( FMAC_FUNCTION_INACTIVE == fmac_RuntimeData.InitState )    )
    {
        const rcc_RequestState_t rccState = Rcc_Set_PeriphActive( RCC_PERIPH_FMAC );

        if( RCC_REQUEST_OK == rccState )
        {
            Fmac_Reset_Runtime();

            fmac_RuntimeData.XferMode             = fmacConfig->XferMode;
            fmac_RuntimeData.DmaPeriphId          = fmacConfig->DmaPeriphId;
            fmac_RuntimeData.DmaWriteChannelId    = fmacConfig->DmaWriteChannelId;
            fmac_RuntimeData.DmaReadChannelId     = fmacConfig->DmaReadChannelId;
            fmac_RuntimeData.DmaPriority          = fmacConfig->DmaPriority;
            fmac_RuntimeData.XferHalfCallback     = fmacConfig->XferHalfCallback;
            fmac_RuntimeData.XferCompleteCallback = fmacConfig->XferCompleteCallback;
            fmac_RuntimeData.ErrorCallback        = fmacConfig->ErrorCallback;

            retState = Fmac_Init_Hardware( fmacConfig->IrqPriority );

            if( FMAC_REQUEST_OK == retState )
            {
                fmac_RuntimeData.InitState = FMAC_FUNCTION_ACTIVE;
            }
            else
            {
                /* Initialization failed, the module is deinitialized */
                (void)Fmac_Release();
                retState = FMAC_REQUEST_ERROR;
            }
        }
        else
        {
            retState = FMAC_REQUEST_ERROR;
        }
    }
    else
    {
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)fmacConfig;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Deinitializes module Fmac
 *
 * Stops the running function and the request, clears the control register, disables the FMAC interrupt in Nvic,
 * resets the unit and disables its clock (Rcc), removes the callbacks and marks the module as not initialized
 * (every other function returns \ref FMAC_REQUEST_ERROR afterwards).
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Deinit( void )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.InitState )
    {
        retState = Fmac_Release();
    }
    else
    {
        /* The module is not initialized */
        retState = FMAC_REQUEST_ERROR;
    }
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Main task of module Fmac
 *
 * In \ref FMAC_XFER_MODE_POLL the request is processed by this function: it moves the input samples to the filter
 * while the input buffer has a free place and the results to the output buffer of the request while the filter has
 * a result. It has to be called periodically. In the other modes the interrupt serves the request and the function
 * does nothing.
 */
void Fmac_Task( void )
{
#if defined(FMAC)
    if( ( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.InitState ) &&
        ( FMAC_XFER_MODE_POLL  == fmac_RuntimeData.XferMode  )    )
    {
        Fmac_Service();
    }
    else
    {
        /* The module is not initialized or the interrupt serves the request */
    }
#endif /* FMAC */
}


/**
 * \brief Tells how many 16-bit words of the local memory a filter configuration needs.
 *
 * The count is the sum of the coefficients, the input buffer and the output buffer (see the memory layout in the
 * description of the module); it is calculated from the configuration only and can exceed
 * \ref FMAC_MEMORY_WORDS (then \ref Fmac_Set_FilterConfig rejects the configuration). The function needs no
 * initialization and touches no register.
 *
 * \param filterConfig [in]:  Pointer to the filter configuration. Must not be NULL.
 * \param wordCnt      [out]: Pointer to store the count of the words. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if the configuration is valid (the function, the
 *         coefficients and their counts, the gain, the clipping, the watermarks, the headroom).
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Get_MemoryUsage( const fmac_FilterConfig_t * const filterConfig,
                                          fmac_SampleCnt_t * const wordCnt )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( FMAC_NULL_PTR != wordCnt )
    {
        retState = Fmac_Check_FilterConfig( filterConfig );

        if( FMAC_REQUEST_OK == retState )
        {
            fmac_MemoryLayout_t layout;

            Fmac_Calc_Layout( filterConfig, &layout );

            *wordCnt = ( layout.X2Size + layout.X1Size + layout.YSize );
        }
        else
        {
            /* Invalid configuration, the count is not written */
        }
    }
    else
    {
        /* Invalid pointer, nothing is written */
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)filterConfig;
    (void)wordCnt;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Configures the filter: lays out the local memory, loads the coefficients and a zero state.
 *
 * The configuration is checked completely (including the memory need and, in DMA mode, the watermarks) before a
 * register is written. The unit is reset, the three buffers are programmed, the clipping is set, the coefficients
 * (X2) are loaded together with a zero state of the input buffer (N - 1 samples) and, for an IIR, of the output
 * buffer (M samples). The coefficients are copied, the arrays of the configuration need not stay valid. A running
 * filter is not reconfigured.
 *
 * \param filterConfig [in]: Pointer to the filter configuration, see \ref fmac_FilterConfig_t. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR (the filter is then not configured).
 */
fmac_RequestState_t Fmac_Set_FilterConfig( const fmac_FilterConfig_t * const filterConfig )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_FUNCTION_ACTIVE != fmac_RuntimeData.InitState   ) ||
        ( FMAC_FILTER_RUNNING   == fmac_RuntimeData.FilterState )    )
    {
        /* The module is not initialized or the filter runs */
        retState = FMAC_REQUEST_ERROR;
    }
    else
    {
        fmac_MemoryLayout_t layout = { 0u };

        retState = Fmac_Check_FilterConfig( filterConfig );

        if( FMAC_REQUEST_OK == retState )
        {
            Fmac_Calc_Layout( filterConfig, &layout );

            if( FMAC_MEMORY_WORDS < ( layout.X2Size + layout.X1Size + layout.YSize ) )
            {
                /* The filter does not fit the local memory */
                retState = FMAC_REQUEST_ERROR;
            }
            else
            {
                /* The memory is sufficient */
            }
        }
        else
        {
            /* Invalid configuration */
        }

        if( ( FMAC_REQUEST_OK    == retState                  ) &&
            ( FMAC_XFER_MODE_DMA == fmac_RuntimeData.XferMode )    )
        {
            if( ( FMAC_WATERMARK_1 != filterConfig->InputWatermark  ) ||
                ( FMAC_WATERMARK_1 != filterConfig->OutputWatermark )    )
            {
                /* The DMA requests need the threshold of one sample */
                retState = FMAC_REQUEST_ERROR;
            }
            else
            {
                /* The watermarks are valid for the DMA requests */
            }
        }
        else
        {
            /* The configuration is invalid or the data are moved by the CPU */
        }

        if( FMAC_REQUEST_OK == retState )
        {
            fmac_RuntimeData.Filter          = *filterConfig;
            fmac_RuntimeData.Layout          = layout;
            fmac_RuntimeData.FilterState     = FMAC_FILTER_UNCONFIGURED;
            fmac_RuntimeData.XferState       = FMAC_XFER_STATE_IDLE;
            fmac_RuntimeData.XferError       = FMAC_XFER_ERROR_NONE;

            Fmac_Build_CoefImage( filterConfig );

            /* The coefficients are copied, the pointers of the application are not kept */
            fmac_RuntimeData.Filter.B = FMAC_NULL_PTR;
            fmac_RuntimeData.Filter.A = FMAC_NULL_PTR;

            retState = Fmac_Program_Filter();

            if( FMAC_REQUEST_OK == retState )
            {
                retState = Fmac_Load_State( FMAC_NULL_PTR, 0u, FMAC_NULL_PTR, 0u );
            }
            else
            {
                /* The programming of the buffers failed */
            }

            if( FMAC_REQUEST_OK == retState )
            {
                fmac_RuntimeData.FilterState = FMAC_FILTER_READY;
            }
            else
            {
                /* The filter stays not configured */
                retState = FMAC_REQUEST_ERROR;
            }
        }
        else
        {
            /* Nothing is written */
        }
    }
#else
    (void)filterConfig;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Starts the filter.
 *
 * Enables the error interrupts (interrupt mode) and starts the hardware function of the FIR or of the IIR filter.
 * The filter has to be configured and preloaded (state \ref FMAC_FILTER_READY).
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Set_FilterStart( void )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.InitState   ) &&
        ( FMAC_FILTER_READY    == fmac_RuntimeData.FilterState )    )
    {
        retState = Fmac_Start_Filter();

        if( FMAC_REQUEST_OK == retState )
        {
            fmac_RuntimeData.FilterState = FMAC_FILTER_RUNNING;
            fmac_RuntimeData.XferState   = FMAC_XFER_STATE_IDLE;
            fmac_RuntimeData.XferError   = FMAC_XFER_ERROR_NONE;
        }
        else
        {
            /* The hardware function did not start, the filter stays ready */
        }
    }
    else
    {
        /* The module is not initialized or the filter is not ready */
        retState = FMAC_REQUEST_ERROR;
    }
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Stops the filter.
 *
 * Ends a request in progress without a callback (the Gpdma channels are stopped in DMA mode), stops the hardware
 * function and disables the interrupts and the DMA requests of the unit. The state of the pipeline of the filter is
 * undefined afterwards, \ref Fmac_Set_FilterPreload has to be called before the filter is started again.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR (also if the filter does not run).
 */
fmac_RequestState_t Fmac_Set_FilterStop( void )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.InitState   ) &&
        ( FMAC_FILTER_RUNNING  == fmac_RuntimeData.FilterState )    )
    {
        retState = Fmac_Stop_Transfer();
    }
    else
    {
        /* The module is not initialized or the filter does not run */
        retState = FMAC_REQUEST_ERROR;
    }
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Loads the initial state of the filter (the history of the input and of the output).
 *
 * The unit is reset (the pointers and the error flags are cleared), the coefficients are loaded again and the input
 * buffer is loaded with N - 1 samples, the output buffer of an IIR with M samples: the given history, oldest sample
 * first and the most recent sample last, behind as many zero samples as the history is shorter than the buffer. A
 * history of \c inputCnt samples is the input before the first sample of the next request.
 *
 * \param inputHistory  [in]: History of the input, may be NULL for \p inputCnt equal to 0.
 * \param inputCnt      [in]: Count of the input history samples, at most N - 1 (0 for a vector function).
 * \param outputHistory [in]: History of the output of an IIR, may be NULL for \p outputCnt equal to 0.
 * \param outputCnt     [in]: Count of the output history samples, at most M (0 for the other functions).
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR. The filter has to be configured and not running.
 */
fmac_RequestState_t Fmac_Set_FilterPreload( const fmac_Sample_t * const inputHistory,
                                            fmac_SampleCnt_t inputCnt,
                                            const fmac_Sample_t * const outputHistory,
                                            fmac_SampleCnt_t outputCnt )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_FUNCTION_ACTIVE      == fmac_RuntimeData.InitState   ) &&
        ( FMAC_FILTER_UNCONFIGURED  != fmac_RuntimeData.FilterState ) &&
        ( FMAC_FILTER_RUNNING       != fmac_RuntimeData.FilterState )    )
    {
        fmac_SampleCnt_t maxInputCnt  = 0u;
        fmac_SampleCnt_t maxOutputCnt = 0u;

        if( ( FMAC_FUNC_FIR == fmac_RuntimeData.Filter.Function ) ||
            ( FMAC_FUNC_IIR == fmac_RuntimeData.Filter.Function )    )
        {
            maxInputCnt = ( fmac_RuntimeData.Filter.BCount - FMAC_HISTORY_OFFSET );
        }
        else
        {
            /* A vector function starts every request from the zero state, no history */
            maxInputCnt = 0u;
        }

        if( FMAC_FUNC_IIR == fmac_RuntimeData.Filter.Function )
        {
            maxOutputCnt = fmac_RuntimeData.Filter.ACount;
        }
        else
        {
            /* Only the IIR has a feedback */
            maxOutputCnt = 0u;
        }

        const fmac_RequestState_t inputState  = Fmac_Check_History( inputHistory, inputCnt, maxInputCnt );
        const fmac_RequestState_t outputState = Fmac_Check_History( outputHistory, outputCnt, maxOutputCnt );

        if( ( FMAC_REQUEST_OK == inputState  ) &&
            ( FMAC_REQUEST_OK == outputState )    )
        {
            retState = Fmac_Reg_Reset_Unit();

            if( FMAC_REQUEST_OK == retState )
            {
                retState = Fmac_Load_State( inputHistory, inputCnt, outputHistory, outputCnt );
            }
            else
            {
                /* The reset of the unit did not finish */
            }

            if( FMAC_REQUEST_OK == retState )
            {
                fmac_RuntimeData.FilterState = FMAC_FILTER_READY;
                fmac_RuntimeData.XferState   = FMAC_XFER_STATE_IDLE;
                fmac_RuntimeData.XferError   = FMAC_XFER_ERROR_NONE;
            }
            else
            {
                /* The state of the filter is undefined */
                fmac_RuntimeData.FilterState = FMAC_FILTER_STOPPED;
                retState = FMAC_REQUEST_ERROR;
            }
        }
        else
        {
            /* Invalid history */
            retState = FMAC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The module is not initialized or the filter is not configured or runs */
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)inputHistory;
    (void)inputCnt;
    (void)outputHistory;
    (void)outputCnt;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Returns the state of the filter.
 *
 * \param filterState [out]: Pointer to store the state of the filter. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR (also if the module is not initialized).
 */
fmac_RequestState_t Fmac_Get_FilterState( fmac_FilterState_t * const filterState )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_NULL_PTR        != filterState                  ) &&
        ( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.InitState   )    )
    {
        *filterState = fmac_RuntimeData.FilterState;

        retState = FMAC_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer or the module is not initialized */
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)filterState;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Starts the processing of a block of samples.
 *
 * The request is processed in the background (see \ref Fmac_Task and the transfer modes), it ends when all results
 * are in the output buffer: the state returns to \ref FMAC_XFER_STATE_IDLE and the complete callback is called (the
 * half callback is called when half of the results are in the buffer). The filter has to run and no request may be in
 * progress. The number of the results follows from the function:
 * - FIR, IIR, convolution and correlation: one result per input sample, \p outputCnt has to be at least \p inputCnt.
 *   The FIR and the IIR continue with the state of the previous request, a convolution and a correlation start from
 *   the zero state in every request (the hardware is started and stopped by the request).
 * - Dot product: exactly N input samples (N is the count of the coefficients) and one result.
 *
 * In DMA mode a request has at most \ref FMAC_DMA_SAMPLE_CNT_MAX samples.
 *
 * \param input     [in]:  Input samples. Must not be NULL.
 * \param inputCnt  [in]:  Count of the input samples, at least 1.
 * \param output    [out]: Buffer of the results. Must not be NULL.
 * \param outputCnt [in]:  Size of the output buffer in samples, at least the count of the results.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Set_FilterProcess( const fmac_Sample_t * const input,
                                            fmac_SampleCnt_t inputCnt,
                                            fmac_Sample_t * const output,
                                            fmac_SampleCnt_t outputCnt )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_FUNCTION_ACTIVE != fmac_RuntimeData.InitState   ) ||
        ( FMAC_FILTER_RUNNING  != fmac_RuntimeData.FilterState ) ||
        ( FMAC_XFER_STATE_IDLE != fmac_RuntimeData.XferState   )    )
    {
        /* The module is not initialized, the filter does not run or a request is in progress */
        retState = FMAC_REQUEST_ERROR;
    }
    else
    {
        fmac_SampleCnt_t feedCnt   = 0u;
        fmac_SampleCnt_t resultCnt = 0u;

        retState = Fmac_Check_Block( input, inputCnt, output, outputCnt, &feedCnt, &resultCnt );

        if( FMAC_REQUEST_OK == retState )
        {
            Fmac_Set_Request( input, feedCnt, output, resultCnt, FMAC_FUNCTION_INACTIVE, FMAC_BUFFER_ONE_SHOT );

            retState = Fmac_Start_Request();

            if( FMAC_REQUEST_OK != retState )
            {
                /* The request did not start, the state of the filter is undefined */
                Fmac_Abort_Request();
                retState = FMAC_REQUEST_ERROR;
            }
            else
            {
                /* The request is in progress */
            }
        }
        else
        {
            /* Invalid parameters or a function which is not a filter */
        }
    }
#else
    (void)input;
    (void)inputCnt;
    (void)output;
    (void)outputCnt;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Starts a stream: the samples of the input buffer are filtered continuously into the output buffer.
 *
 * Only the FIR and the IIR filter stream. The input buffer and the output buffer have the same size (at least 2
 * samples, in DMA mode at most \ref FMAC_DMA_SAMPLE_CNT_MAX), one result per sample. In
 * \ref FMAC_BUFFER_CIRCULAR mode the input buffer is read and the output buffer is written again from the begin until
 * \ref Fmac_Set_StreamStop (the input buffer is filled by the application or by another DMA channel, the module does
 * not wait for new samples); the half callback is called when half of the output buffer is filled, the complete
 * callback when it is full (at the end of every lap). In \ref FMAC_BUFFER_ONE_SHOT mode the stream ends after one
 * pass: the state returns to \ref FMAC_XFER_STATE_IDLE and the complete callback is called. The filter has to run and
 * no request may be in progress.
 *
 * \param streamRequest [in]: Pointer to the stream request. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Set_StreamStart( const fmac_StreamRequest_t * const streamRequest )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_FUNCTION_ACTIVE != fmac_RuntimeData.InitState   ) ||
        ( FMAC_FILTER_RUNNING  != fmac_RuntimeData.FilterState ) ||
        ( FMAC_XFER_STATE_IDLE != fmac_RuntimeData.XferState   )    )
    {
        /* The module is not initialized, the filter does not run or a request is in progress */
        retState = FMAC_REQUEST_ERROR;
    }
    else
    {
        retState = Fmac_Check_Stream( streamRequest );

        if( FMAC_REQUEST_OK == retState )
        {
            Fmac_Set_Request( streamRequest->Input,
                              streamRequest->InputSize,
                              streamRequest->Output,
                              streamRequest->OutputSize,
                              FMAC_FUNCTION_ACTIVE,
                              streamRequest->BufferMode );

            retState = Fmac_Start_Request();

            if( FMAC_REQUEST_OK != retState )
            {
                /* The stream did not start, the state of the filter is undefined */
                Fmac_Abort_Request();
                retState = FMAC_REQUEST_ERROR;
            }
            else
            {
                /* The stream is in progress */
            }
        }
        else
        {
            /* Invalid request or a function which does not stream */
        }
    }
#else
    (void)streamRequest;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Stops a stream (also an unfinished one-shot stream).
 *
 * The Gpdma channels are stopped in DMA mode, the hardware function is stopped and the interrupts and the DMA requests
 * of the unit are disabled, no callback is called. Samples of the stream are still in the pipeline of the filter, so
 * the filter is \ref FMAC_FILTER_STOPPED and \ref Fmac_Set_FilterPreload is required before it is started again.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR (also if no stream is in progress).
 */
fmac_RequestState_t Fmac_Set_StreamStop( void )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.InitState ) &&
        ( FMAC_XFER_STATE_BUSY == fmac_RuntimeData.XferState ) &&
        ( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.ReqStream )    )
    {
        retState = Fmac_Stop_Transfer();
    }
    else
    {
        /* The module is not initialized or no stream is in progress */
        retState = FMAC_REQUEST_ERROR;
    }
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Returns the state of the processing of the data.
 *
 * \param xferState [out]: Pointer to store the state. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR (also if the module is not initialized).
 */
fmac_RequestState_t Fmac_Get_XferState( fmac_XferState_t * const xferState )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_NULL_PTR        != xferState                  ) &&
        ( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.InitState )    )
    {
        *xferState = fmac_RuntimeData.XferState;

        retState = FMAC_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer or the module is not initialized */
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)xferState;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Returns the errors of the current or of the last request (bit mask of \ref fmac_XferError_t).
 *
 * \param xferError [out]: Pointer to store the errors. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR (also if the module is not initialized).
 */
fmac_RequestState_t Fmac_Get_XferError( fmac_XferError_t * const xferError )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_NULL_PTR        != xferError                  ) &&
        ( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.InitState )    )
    {
        *xferError = fmac_RuntimeData.XferError;

        retState = FMAC_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer or the module is not initialized */
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)xferError;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Converts a floating point value to a q1.15 sample (rounded to the nearest, saturated to the q1.15 range).
 *
 * \param value  [in]:  Value to convert. A NaN is rejected.
 * \param sample [out]: Pointer to store the sample. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Get_FromFloat( float value, fmac_Sample_t * const sample )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_NULL_PTR != sample ) &&
        ( value         == value  )    )
    {
        const float scaledValue = ( value * FMAC_Q15_SCALE );

        if( FMAC_Q15_MAX_FLOAT <= scaledValue )
        {
            *sample = FMAC_Q15_MAX;
        }
        else if( FMAC_Q15_MIN_FLOAT >= scaledValue )
        {
            *sample = FMAC_Q15_MIN;
        }
        else if( FMAC_Q15_ZERO <= scaledValue )
        {
            *sample = (fmac_Sample_t)( scaledValue + FMAC_Q15_HALF );
        }
        else
        {
            *sample = (fmac_Sample_t)( scaledValue - FMAC_Q15_HALF );
        }

        retState = FMAC_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer or not a number */
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)value;
    (void)sample;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Converts a q1.15 sample to a floating point value.
 *
 * \param sample [in]:  Sample to convert.
 * \param value  [out]: Pointer to store the value. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Get_ToFloat( fmac_Sample_t sample, float * const value )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( FMAC_NULL_PTR != value )
    {
        *value = ( (float)sample / FMAC_Q15_SCALE );

        retState = FMAC_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer */
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)sample;
    (void)value;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Sets the priority of the FMAC interrupt (Nvic).
 *
 * \param irqPrio [in]: Interrupt priority.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR (also if the module is not initialized).
 */
fmac_RequestState_t Fmac_Set_IrqPriority( fmac_IrqPrio_t irqPrio )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.InitState )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_FMAC, irqPrio );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = FMAC_REQUEST_OK;
        }
        else
        {
            retState = FMAC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The module is not initialized */
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)irqPrio;
#endif /* FMAC */

    return ( retState );
}


/**
 * \brief Gets the priority of the FMAC interrupt (Nvic).
 *
 * \param irqPrio [out]: Pointer to store the interrupt priority. Must not be NULL.
 *
 * \return Function processing state. Returns \ref FMAC_REQUEST_OK if request was processed without problems.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR (also if the module is not initialized).
 */
fmac_RequestState_t Fmac_Get_IrqPriority( fmac_IrqPrio_t * const irqPrio )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

#if defined(FMAC)
    if( ( FMAC_NULL_PTR        != irqPrio                    ) &&
        ( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.InitState )    )
    {
        const nvic_RequestState_t nvicState = Nvic_Get_PeriphIrq_Prio( NVIC_PERIPH_IRQ_FMAC, irqPrio );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = FMAC_REQUEST_OK;
        }
        else
        {
            retState = FMAC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid pointer or the module is not initialized */
        retState = FMAC_REQUEST_ERROR;
    }
#else
    (void)irqPrio;
#endif /* FMAC */

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

#if defined(FMAC)

/**
 * \brief Checks the configuration of the module: the pointer, the transfer mode and, in DMA mode, the Gpdma
 *        controller, the two different channels and the priority.
 *
 * \param fmacConfig [in]: Pointer to the configuration.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the configuration is valid. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Check_Config( const fmac_Config_t * const fmacConfig )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

    if( ( FMAC_NULL_PTR     != fmacConfig            ) &&
        ( FMAC_XFER_MODE_CNT > fmacConfig->XferMode  )    )
    {
        if( FMAC_XFER_MODE_DMA == fmacConfig->XferMode )
        {
            if( ( GPDMA_PERIPH_CNT   >  fmacConfig->DmaPeriphId       ) &&
                ( GPDMA_CHANNEL_CNT  >  fmacConfig->DmaWriteChannelId ) &&
                ( GPDMA_CHANNEL_CNT  >  fmacConfig->DmaReadChannelId  ) &&
                ( GPDMA_PRIORITY_CNT >  fmacConfig->DmaPriority       ) &&
                ( fmacConfig->DmaWriteChannelId != fmacConfig->DmaReadChannelId )    )
            {
                retState = FMAC_REQUEST_OK;
            }
            else
            {
                /* Invalid DMA configuration */
                retState = FMAC_REQUEST_ERROR;
            }
        }
        else
        {
            retState = FMAC_REQUEST_OK;
        }
    }
    else
    {
        /* Invalid pointer or transfer mode */
        retState = FMAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Tells whether the configured function is a vector function (dot product, convolution, correlation): its
 *        request starts the hardware function from the zero state, the FIR and the IIR run from the start of the
 *        filter.
 *
 * \return \ref FMAC_FUNCTION_ACTIVE for a vector function, otherwise \ref FMAC_FUNCTION_INACTIVE.
 */
static fmac_FunctionState_t Fmac_Is_VectorFunction( void )
{
    fmac_FunctionState_t retState = FMAC_FUNCTION_ACTIVE;

    if( FMAC_FUNC_FIR == fmac_RuntimeData.Filter.Function )
    {
        retState = FMAC_FUNCTION_INACTIVE;
    }
    else if( FMAC_FUNC_IIR == fmac_RuntimeData.Filter.Function )
    {
        retState = FMAC_FUNCTION_INACTIVE;
    }
    else
    {
        /* Dot product, convolution and correlation are vector functions */
        retState = FMAC_FUNCTION_ACTIVE;
    }

    return ( retState );
}


/**
 * \brief Tells whether the request in progress is a circular stream.
 *
 * \return \ref FMAC_FUNCTION_ACTIVE for a circular stream, otherwise \ref FMAC_FUNCTION_INACTIVE.
 */
static fmac_FunctionState_t Fmac_Is_Circular( void )
{
    fmac_FunctionState_t retState = FMAC_FUNCTION_INACTIVE;

    if( ( FMAC_FUNCTION_ACTIVE  == fmac_RuntimeData.ReqStream     ) &&
        ( FMAC_BUFFER_CIRCULAR  == fmac_RuntimeData.ReqBufferMode )    )
    {
        retState = FMAC_FUNCTION_ACTIVE;
    }
    else
    {
        /* A block request or a one shot stream */
        retState = FMAC_FUNCTION_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Checks a filter configuration: the function, the pointers, the coefficient counts of the function, the
 *        gain, the clipping, the watermarks and the headroom.
 *
 * \param filterConfig [in]: Pointer to the filter configuration.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the configuration is valid. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Check_FilterConfig( const fmac_FilterConfig_t * const filterConfig )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

    if( ( FMAC_NULL_PTR     != filterConfig                  ) &&
        ( FMAC_FUNC_CNT      > filterConfig->Function        ) &&
        ( FMAC_NULL_PTR     != filterConfig->B               ) &&
        ( FMAC_GAIN_CNT      > filterConfig->Gain            ) &&
        ( FMAC_CLIP_CNT      > filterConfig->Clipping        ) &&
        ( FMAC_WATERMARK_CNT > filterConfig->InputWatermark  ) &&
        ( FMAC_WATERMARK_CNT > filterConfig->OutputWatermark ) &&
        ( FMAC_MEMORY_WORDS >= filterConfig->InputHeadroom   ) &&
        ( FMAC_MEMORY_WORDS >= filterConfig->OutputHeadroom  )    )
    {
        if( FMAC_FUNC_IIR == filterConfig->Function )
        {
            if( ( FMAC_IIR_FEEDFORWARD_MIN <= filterConfig->BCount ) &&
                ( FMAC_IIR_FEEDFORWARD_MAX >= filterConfig->BCount ) &&
                ( FMAC_NULL_PTR            != filterConfig->A      ) &&
                ( FMAC_IIR_FEEDBACK_MIN    <= filterConfig->ACount ) &&
                ( FMAC_IIR_FEEDBACK_MAX    >= filterConfig->ACount )    )
            {
                retState = FMAC_REQUEST_OK;
            }
            else
            {
                /* Coefficients of the IIR out of the range of the hardware */
                retState = FMAC_REQUEST_ERROR;
            }
        }
        else
        {
            if( ( FMAC_FIR_COEF_MIN <= filterConfig->BCount ) &&
                ( FMAC_FIR_COEF_MAX >= filterConfig->BCount )    )
            {
                retState = FMAC_REQUEST_OK;
            }
            else
            {
                /* Coefficients out of the range of the hardware */
                retState = FMAC_REQUEST_ERROR;
            }
        }
    }
    else
    {
        /* Invalid pointer or parameter */
        retState = FMAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks a history of a preload: the count is within the limit and the pointer is valid for a non empty history.
 *
 * \param history    [in]: Pointer to the history, may be NULL for an empty history.
 * \param historyCnt [in]: Count of the history samples.
 * \param maxCnt     [in]: Largest accepted count.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the history is valid. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Check_History( const fmac_Sample_t * const history,
                                               fmac_SampleCnt_t historyCnt,
                                               fmac_SampleCnt_t maxCnt )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

    if( maxCnt >= historyCnt )
    {
        if( 0u == historyCnt )
        {
            /* No history, the buffer is loaded with zero samples */
            retState = FMAC_REQUEST_OK;
        }
        else if( FMAC_NULL_PTR != history )
        {
            retState = FMAC_REQUEST_OK;
        }
        else
        {
            /* A history without samples */
            retState = FMAC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The history is longer than the buffer */
        retState = FMAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks the parameters of a block request and calculates the count of the samples to write to the filter
 *        and the count of the results.
 *
 * \param input     [in]:  Input samples.
 * \param inputCnt  [in]:  Count of the input samples.
 * \param output    [in]:  Buffer of the results.
 * \param outputCnt [in]:  Size of the output buffer in samples.
 * \param feedCnt   [out]: Pointer to store the count of the samples written to the filter by the request (0 for a dot
 *                         product, its inputs are preloaded).
 * \param resultCnt [out]: Pointer to store the count of the results.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the request is valid. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Check_Block( const fmac_Sample_t * const input,
                                             fmac_SampleCnt_t inputCnt,
                                             fmac_Sample_t * const output,
                                             fmac_SampleCnt_t outputCnt,
                                             fmac_SampleCnt_t * const feedCnt,
                                             fmac_SampleCnt_t * const resultCnt )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

    if( ( FMAC_NULL_PTR != input     ) &&
        ( FMAC_NULL_PTR != output    ) &&
        ( 0u             < inputCnt  ) &&
        ( 0u             < outputCnt )    )
    {
        if( FMAC_FUNC_DOT_PRODUCT == fmac_RuntimeData.Filter.Function )
        {
            if( fmac_RuntimeData.Filter.BCount == inputCnt )
            {
                *feedCnt   = 0u;
                *resultCnt = FMAC_DOT_RESULT_CNT;
                retState   = FMAC_REQUEST_OK;
            }
            else
            {
                /* The dot product takes exactly the count of the coefficients */
                retState = FMAC_REQUEST_ERROR;
            }
        }
        else if( inputCnt <= outputCnt )
        {
            *feedCnt   = inputCnt;
            *resultCnt = inputCnt;
            retState   = FMAC_REQUEST_OK;
        }
        else
        {
            /* The output buffer is smaller than the count of the results */
            retState = FMAC_REQUEST_ERROR;
        }

        if( ( FMAC_REQUEST_OK    == retState                  ) &&
            ( FMAC_XFER_MODE_DMA == fmac_RuntimeData.XferMode )    )
        {
            if( FMAC_DMA_SAMPLE_CNT_MAX < inputCnt )
            {
                /* A Gpdma block is limited */
                retState = FMAC_REQUEST_ERROR;
            }
            else
            {
                /* The request fits a Gpdma block */
            }
        }
        else
        {
            /* The request is invalid or no DMA limit applies */
        }
    }
    else
    {
        /* Invalid pointer or count */
        retState = FMAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks a stream request: the pointers, equal sizes of at least \ref FMAC_STREAM_SIZE_MIN samples (in DMA mode
 *        at most \ref FMAC_DMA_SAMPLE_CNT_MAX), the buffer mode and a function which streams (FIR, IIR).
 *
 * \param streamRequest [in]: Pointer to the stream request.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the request is valid. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Check_Stream( const fmac_StreamRequest_t * const streamRequest )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

    if( ( FMAC_NULL_PTR        != streamRequest                                    ) &&
        ( FMAC_NULL_PTR        != streamRequest->Input                             ) &&
        ( FMAC_NULL_PTR        != streamRequest->Output                            ) &&
        ( FMAC_BUFFER_CNT       > streamRequest->BufferMode                        ) &&
        ( FMAC_STREAM_SIZE_MIN <= streamRequest->InputSize                         ) &&
        ( streamRequest->InputSize == streamRequest->OutputSize                    )    )
    {
        const fmac_FunctionState_t vector = Fmac_Is_VectorFunction();

        if( FMAC_FUNCTION_INACTIVE == vector )
        {
            retState = FMAC_REQUEST_OK;
        }
        else
        {
            /* The vector functions do not stream */
            retState = FMAC_REQUEST_ERROR;
        }

        if( ( FMAC_REQUEST_OK    == retState                  ) &&
            ( FMAC_XFER_MODE_DMA == fmac_RuntimeData.XferMode )    )
        {
            if( FMAC_DMA_SAMPLE_CNT_MAX < streamRequest->InputSize )
            {
                /* A Gpdma block is limited */
                retState = FMAC_REQUEST_ERROR;
            }
            else
            {
                /* The stream fits a Gpdma block */
            }
        }
        else
        {
            /* The request is invalid or no DMA limit applies */
        }
    }
    else
    {
        /* Invalid pointer, size or buffer mode */
        retState = FMAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Calculates the layout of the local memory of a valid filter configuration.
 *
 * \param filterConfig [in]:  Pointer to the valid filter configuration.
 * \param layout       [out]: Pointer to store the layout.
 */
static void Fmac_Calc_Layout( const fmac_FilterConfig_t * const filterConfig,
                              fmac_MemoryLayout_t * const layout )
{
    fmac_SampleCnt_t feedbackCnt = 0u;

    if( FMAC_FUNC_IIR == filterConfig->Function )
    {
        feedbackCnt = filterConfig->ACount;
    }
    else
    {
        /* Only the IIR has the feedback coefficients */
        feedbackCnt = 0u;
    }

    layout->X2Base = 0u;
    layout->X2Size = ( filterConfig->BCount + feedbackCnt );
    layout->X1Base = ( layout->X2Base + layout->X2Size );
    layout->X1Size = ( filterConfig->BCount +
                       Fmac_Get_Headroom( filterConfig->InputHeadroom, filterConfig->InputWatermark ) );
    layout->YBase  = ( layout->X1Base + layout->X1Size );
    layout->YSize  = ( feedbackCnt +
                       Fmac_Get_Headroom( filterConfig->OutputHeadroom, filterConfig->OutputWatermark ) );
}


/**
 * \brief Returns the headroom of a circular buffer: the larger one of the requested count and of the watermark
 *        threshold (the hardware requires the threshold not above the headroom).
 *
 * \param requestedCnt [in]: Requested count of the spare samples.
 * \param watermark    [in]: Watermark of the buffer.
 *
 * \return Count of the spare samples of the buffer.
 */
static fmac_SampleCnt_t Fmac_Get_Headroom( fmac_SampleCnt_t requestedCnt, fmac_Watermark_t watermark )
{
    fmac_SampleCnt_t retCnt = ( FMAC_WATERMARK_UNIT << (uint32_t)watermark );

    if( retCnt < requestedCnt )
    {
        retCnt = requestedCnt;
    }
    else
    {
        /* The threshold of the watermark is the larger one */
    }

    return ( retCnt );
}


/**
 * \brief Builds the image of the X2 buffer in the runtime data: the feedforward coefficients followed by the
 *        feedback coefficients of an IIR; the coefficients of a correlation and of a dot product reversed (the
 *        hardware convolution then calculates the correlation).
 *
 * \param filterConfig [in]: Pointer to the valid filter configuration.
 */
static void Fmac_Build_CoefImage( const fmac_FilterConfig_t * const filterConfig )
{
    const fmac_SampleCnt_t feedforwardCnt = filterConfig->BCount;
    const fmac_SampleCnt_t lastCoefIdx    = ( feedforwardCnt - FMAC_HISTORY_OFFSET );

    if( ( FMAC_FUNC_CORRELATION == filterConfig->Function ) ||
        ( FMAC_FUNC_DOT_PRODUCT == filterConfig->Function )    )
    {
        for( fmac_SampleCnt_t coefIdx = 0u; feedforwardCnt > coefIdx; coefIdx ++ )
        {
            fmac_RuntimeData.CoefImage[ coefIdx ] = filterConfig->B[ lastCoefIdx - coefIdx ];
        }
    }
    else
    {
        for( fmac_SampleCnt_t coefIdx = 0u; feedforwardCnt > coefIdx; coefIdx ++ )
        {
            fmac_RuntimeData.CoefImage[ coefIdx ] = filterConfig->B[ coefIdx ];
        }
    }

    if( FMAC_FUNC_IIR == filterConfig->Function )
    {
        for( fmac_SampleCnt_t coefIdx = 0u; filterConfig->ACount > coefIdx; coefIdx ++ )
        {
            fmac_RuntimeData.CoefImage[ feedforwardCnt + coefIdx ] = filterConfig->A[ coefIdx ];
        }
    }
    else
    {
        /* Only the IIR has the feedback coefficients */
    }
}


/**
 * \brief Resets the unit and programs the three buffers of the stored layout and the clipping.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the unit was reset. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Program_Filter( void )
{
    fmac_RequestState_t retState = Fmac_Reg_Reset_Unit();

    if( FMAC_REQUEST_OK == retState )
    {
        const fmac_RegBufCfg_t x1Cfg =
        {
            .Base      = fmac_RuntimeData.Layout.X1Base,
            .Size      = fmac_RuntimeData.Layout.X1Size,
            .Watermark = (uint32_t)fmac_RuntimeData.Filter.InputWatermark,
        };
        const fmac_RegBufCfg_t x2Cfg =
        {
            .Base      = fmac_RuntimeData.Layout.X2Base,
            .Size      = fmac_RuntimeData.Layout.X2Size,
            .Watermark = 0u,
        };
        const fmac_RegBufCfg_t yCfg =
        {
            .Base      = fmac_RuntimeData.Layout.YBase,
            .Size      = fmac_RuntimeData.Layout.YSize,
            .Watermark = (uint32_t)fmac_RuntimeData.Filter.OutputWatermark,
        };

        Fmac_Reg_Set_BufCfg( FMAC_REG_X1BUFCFG, &x1Cfg );
        Fmac_Reg_Set_BufCfg( FMAC_REG_X2BUFCFG, &x2Cfg );
        Fmac_Reg_Set_BufCfg( FMAC_REG_YBUFCFG, &yCfg );

        if( FMAC_CLIP_SATURATE == fmac_RuntimeData.Filter.Clipping )
        {
            Fmac_Set_RegBit( FMAC_REG_CR, FMAC_CR_CLIPEN );
        }
        else
        {
            Fmac_Reset_RegBit( FMAC_REG_CR, FMAC_CR_CLIPEN );
        }
    }
    else
    {
        /* The reset of the unit did not finish */
    }

    return ( retState );
}


/**
 * \brief Loads the coefficients and the state of the filter into the local memory (the unit is reset and no
 *        function runs): X2 with the coefficient image, X1 with N - 1 samples and the Y buffer of an IIR with M
 *        samples, the history behind the zero samples. A vector function loads no state, it is loaded by every
 *        request.
 *
 * \param inputHistory  [in]: History of the input, valid for \p inputCnt greater than 0.
 * \param inputCnt      [in]: Count of the input history samples.
 * \param outputHistory [in]: History of the output, valid for \p outputCnt greater than 0.
 * \param outputCnt     [in]: Count of the output history samples.
 *
 * \return Returns \ref FMAC_REQUEST_OK if all functions completed. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Load_State( const fmac_Sample_t * const inputHistory,
                                            fmac_SampleCnt_t inputCnt,
                                            const fmac_Sample_t * const outputHistory,
                                            fmac_SampleCnt_t outputCnt )
{
    fmac_RequestState_t         retState    = FMAC_REQUEST_ERROR;
    const fmac_SampleCnt_t      feedfwdCnt  = fmac_RuntimeData.Filter.BCount;
    fmac_SampleCnt_t            feedbackCnt = 0u;
    fmac_RegFuncParam_t         funcParam;

    if( FMAC_FUNC_IIR == fmac_RuntimeData.Filter.Function )
    {
        feedbackCnt = fmac_RuntimeData.Filter.ACount;
    }
    else
    {
        /* Only the IIR has the feedback coefficients */
        feedbackCnt = 0u;
    }

    const fmac_SampleCnt_t coefCnt = ( feedfwdCnt + feedbackCnt );

    funcParam.Func   = FMAC_REG_FUNC_LOAD_X2;
    funcParam.ParamP = feedfwdCnt;
    funcParam.ParamQ = feedbackCnt;
    funcParam.ParamR = 0u;

    retState = Fmac_Load_Buffer( &funcParam, coefCnt, fmac_RuntimeData.CoefImage, coefCnt );

    if( ( FMAC_REQUEST_OK == retState ) &&
        ( ( FMAC_FUNC_FIR == fmac_RuntimeData.Filter.Function ) ||
          ( FMAC_FUNC_IIR == fmac_RuntimeData.Filter.Function )    ) )
    {
        funcParam.Func   = FMAC_REG_FUNC_LOAD_X1;
        funcParam.ParamP = ( feedfwdCnt - FMAC_HISTORY_OFFSET );
        funcParam.ParamQ = 0u;

        retState = Fmac_Load_Buffer( &funcParam, ( feedfwdCnt - FMAC_HISTORY_OFFSET ), inputHistory, inputCnt );
    }
    else
    {
        /* The coefficients were not loaded or a vector function loads no state */
    }

    if( ( FMAC_REQUEST_OK == retState                   ) &&
        ( FMAC_FUNC_IIR   == fmac_RuntimeData.Filter.Function ) )
    {
        funcParam.Func   = FMAC_REG_FUNC_LOAD_Y;
        funcParam.ParamP = feedbackCnt;
        funcParam.ParamQ = 0u;

        retState = Fmac_Load_Buffer( &funcParam, feedbackCnt, outputHistory, outputCnt );
    }
    else
    {
        /* A previous step failed or only the IIR has a feedback state */
    }

    return ( retState );
}


/**
 * \brief Runs one load function: starts it, writes \p totalCnt values (zero samples followed by the \p valueCnt given
 *        values) and waits until the hardware completes the function.
 *
 * \param funcParam [in]: Load function and its parameters.
 * \param totalCnt  [in]: Count of the values the function takes.
 * \param values    [in]: Values written behind the zero samples, valid for \p valueCnt greater than 0.
 * \param valueCnt  [in]: Count of the given values, at most \p totalCnt.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the function completed. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Load_Buffer( const fmac_RegFuncParam_t * const funcParam,
                                             fmac_SampleCnt_t totalCnt,
                                             const fmac_Sample_t * const values,
                                             fmac_SampleCnt_t valueCnt )
{
    const fmac_SampleCnt_t zeroCnt = ( totalCnt - valueCnt );

    Fmac_Reg_Start_Function( funcParam );

    for( fmac_SampleCnt_t valueIdx = 0u; totalCnt > valueIdx; valueIdx ++ )
    {
        if( zeroCnt > valueIdx )
        {
            Fmac_Reg_Write_Sample( 0 );
        }
        else
        {
            Fmac_Reg_Write_Sample( values[ valueIdx - zeroCnt ] );
        }
    }

    return ( Fmac_Reg_Wait_FunctionDone() );
}


/**
 * \brief Enables the error interrupts (interrupt and DMA mode) and, for the FIR and the IIR, the DMA requests (DMA
 *        mode) and starts the hardware function. A vector function starts the hardware by every request.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the function started. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Start_Filter( void )
{
    fmac_RequestState_t        retState = FMAC_REQUEST_OK;
    const fmac_FunctionState_t vector   = Fmac_Is_VectorFunction();

    if( ( FMAC_XFER_MODE_ISR == fmac_RuntimeData.XferMode ) ||
        ( FMAC_XFER_MODE_DMA == fmac_RuntimeData.XferMode )    )
    {
        Fmac_Enable_ErrorIrqs();
    }
    else
    {
        /* The errors are checked by Fmac_Task */
    }

    if( FMAC_FUNCTION_INACTIVE == vector )
    {
        if( FMAC_XFER_MODE_DMA == fmac_RuntimeData.XferMode )
        {
            /* The DMA requests are enabled before the function starts, the channels are armed by the requests */
            Fmac_Set_RegBit( FMAC_REG_CR, ( FMAC_CR_DMAWEN | FMAC_CR_DMAREN ) );
        }
        else
        {
            /* The data are moved by the CPU */
        }

        retState = Fmac_Run_Function();
    }
    else
    {
        /* A vector function starts the hardware by every request */
    }

    return ( retState );
}


/**
 * \brief Starts the hardware function of the configured filter (the FIR function of the hardware for a vector
 *        function) with the coefficient counts and the gain and waits until the function and its parameters read back
 *        as written (the START bit is not checked: a vector function whose request is read out at once by the
 *        interrupt or by the Gpdma is stopped by the end of the request before the check).
 *
 * \return Returns \ref FMAC_REQUEST_OK if the function started. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Run_Function( void )
{
    fmac_RegFuncParam_t funcParam;

    if( FMAC_FUNC_IIR == fmac_RuntimeData.Filter.Function )
    {
        funcParam.Func   = FMAC_REG_FUNC_IIR;
        funcParam.ParamQ = fmac_RuntimeData.Filter.ACount;
    }
    else
    {
        funcParam.Func   = FMAC_REG_FUNC_FIR;
        funcParam.ParamQ = 0u;
    }

    funcParam.ParamP = fmac_RuntimeData.Filter.BCount;
    funcParam.ParamR = (uint32_t)fmac_RuntimeData.Filter.Gain;

    Fmac_Reg_Start_Function( &funcParam );

    return ( Fmac_Reg_Wait_FunctionParam( &funcParam ) );
}


/**
 * \brief Enables the error interrupts of the unit for the flags which are not set (a set flag stays set until the
 *        unit is reset and would keep the interrupt asserted).
 */
static void Fmac_Enable_ErrorIrqs( void )
{
    const uint32_t errorFlags = Fmac_Get_RegVal( FMAC_REG_SR, FMAC_SR_ERROR_MASK );
    uint32_t       enableMask = FMAC_CR_ERROR_IRQ_MASK;

    if( FMAC_BITS_CLEAR != ( errorFlags & FMAC_SR_SAT ) )
    {
        enableMask &= ~FMAC_CR_SATIEN;
    }
    else
    {
        /* The saturation flag is not set */
    }

    if( FMAC_BITS_CLEAR != ( errorFlags & FMAC_SR_UNFL ) )
    {
        enableMask &= ~FMAC_CR_UNFLIEN;
    }
    else
    {
        /* The underflow flag is not set */
    }

    if( FMAC_BITS_CLEAR != ( errorFlags & FMAC_SR_OVFL ) )
    {
        enableMask &= ~FMAC_CR_OVFLIEN;
    }
    else
    {
        /* The overflow flag is not set */
    }

    Fmac_Set_RegBit( FMAC_REG_CR, enableMask );
}


/**
 * \brief Stores a request in the runtime data and sets the state busy.
 *
 * \param input      [in]: Input samples (the preloaded inputs of a dot product).
 * \param feedCnt    [in]: Count of the samples written to the filter by the request (the size of the input buffer
 *                         of a stream).
 * \param output     [in]: Buffer of the results.
 * \param resultCnt  [in]: Count of the results (the size of the output buffer of a stream).
 * \param stream     [in]: \ref FMAC_FUNCTION_ACTIVE for a stream, \ref FMAC_FUNCTION_INACTIVE for a block request.
 * \param bufferMode [in]: Buffer mode of a stream (one shot for a block request).
 */
static void Fmac_Set_Request( const fmac_Sample_t * const input,
                              fmac_SampleCnt_t feedCnt,
                              fmac_Sample_t * const output,
                              fmac_SampleCnt_t resultCnt,
                              fmac_FunctionState_t stream,
                              fmac_BufferMode_t bufferMode )
{
    fmac_RuntimeData.ReqInput      = input;
    fmac_RuntimeData.ReqOutput     = output;
    fmac_RuntimeData.ReqInputCnt   = feedCnt;
    fmac_RuntimeData.ReqOutputCnt  = resultCnt;
    fmac_RuntimeData.ReqInputIdx   = 0u;
    fmac_RuntimeData.ReqOutputIdx  = 0u;
    fmac_RuntimeData.ReqHalfIdx    = ( resultCnt / FMAC_HALF_DIVISOR );
    fmac_RuntimeData.ReqStream     = stream;
    fmac_RuntimeData.ReqBufferMode = bufferMode;
    fmac_RuntimeData.XferError     = FMAC_XFER_ERROR_NONE;
    fmac_RuntimeData.XferState     = FMAC_XFER_STATE_BUSY;
}


/**
 * \brief Starts the request stored in the runtime data: a vector function loads its state and the hardware function
 *        starts with the request; the interrupts (interrupt mode) or the Gpdma channels (DMA mode) are enabled.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the request started. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Start_Request( void )
{
    fmac_RequestState_t        retState = FMAC_REQUEST_OK;
    const fmac_FunctionState_t vector   = Fmac_Is_VectorFunction();

    if( FMAC_FUNCTION_ACTIVE == vector )
    {
        retState = Fmac_Arm_Vector();
    }
    else
    {
        /* The FIR and the IIR run since the start of the filter */
    }

    if( FMAC_REQUEST_OK == retState )
    {
        fmac_RuntimeData.ErrorBaseline = Fmac_Get_HwErrors();

        if( FMAC_XFER_MODE_ISR == fmac_RuntimeData.XferMode )
        {
            Fmac_Enable_ErrorIrqs();
            Fmac_Set_RegBit( FMAC_REG_CR, FMAC_CR_RIEN );

            if( 0u < fmac_RuntimeData.ReqInputCnt )
            {
                Fmac_Set_RegBit( FMAC_REG_CR, FMAC_CR_WIEN );
            }
            else
            {
                /* No sample to write, the write interrupt would stay asserted */
            }
        }
        else if( FMAC_XFER_MODE_DMA == fmac_RuntimeData.XferMode )
        {
            Fmac_Enable_ErrorIrqs();
            retState = Fmac_Dma_Start();
        }
        else
        {
            /* The request is processed by Fmac_Task */
        }
    }
    else
    {
        /* The state of the vector function was not loaded */
    }

    if( ( FMAC_REQUEST_OK     == retState ) &&
        ( FMAC_FUNCTION_ACTIVE == vector   )    )
    {
        retState = Fmac_Run_Function();
    }
    else
    {
        /* A previous step failed or the FIR / IIR function runs */
    }

    return ( retState );
}


/**
 * \brief Prepares the hardware for the request of a vector function: resets the unit, loads the coefficients and the
 *        state (N - 1 zero samples, the N inputs of a dot product) and enables the DMA requests (DMA mode). The
 *        hardware function is started by the caller.
 *
 * \return Returns \ref FMAC_REQUEST_OK if every load function completed. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Arm_Vector( void )
{
    fmac_RequestState_t retState = Fmac_Reg_Reset_Unit();

    if( FMAC_REQUEST_OK == retState )
    {
        /* A vector function loads the coefficients only, the state is loaded below */
        retState = Fmac_Load_State( FMAC_NULL_PTR, 0u, FMAC_NULL_PTR, 0u );
    }
    else
    {
        /* The reset of the unit did not finish */
    }

    if( FMAC_REQUEST_OK == retState )
    {
        const fmac_SampleCnt_t feedfwdCnt = fmac_RuntimeData.Filter.BCount;
        fmac_RegFuncParam_t    funcParam;

        funcParam.Func   = FMAC_REG_FUNC_LOAD_X1;
        funcParam.ParamQ = 0u;
        funcParam.ParamR = 0u;

        if( FMAC_FUNC_DOT_PRODUCT == fmac_RuntimeData.Filter.Function )
        {
            funcParam.ParamP = feedfwdCnt;

            retState = Fmac_Load_Buffer( &funcParam, feedfwdCnt, fmac_RuntimeData.ReqInput, feedfwdCnt );
        }
        else
        {
            funcParam.ParamP = ( feedfwdCnt - FMAC_HISTORY_OFFSET );

            retState = Fmac_Load_Buffer( &funcParam, ( feedfwdCnt - FMAC_HISTORY_OFFSET ), FMAC_NULL_PTR, 0u );
        }
    }
    else
    {
        /* A previous step failed */
    }

    if( ( FMAC_REQUEST_OK    == retState                  ) &&
        ( FMAC_XFER_MODE_DMA == fmac_RuntimeData.XferMode )    )
    {
        /* The function is not running: the DMA request enables can be changed */
        Fmac_Set_RegBit( FMAC_REG_CR, FMAC_CR_DMAREN );

        if( 0u < fmac_RuntimeData.ReqInputCnt )
        {
            Fmac_Set_RegBit( FMAC_REG_CR, FMAC_CR_DMAWEN );
        }
        else
        {
            /* A dot product writes no sample, no write request */
            Fmac_Reset_RegBit( FMAC_REG_CR, FMAC_CR_DMAWEN );
        }
    }
    else
    {
        /* A previous step failed or the data are moved by the CPU */
    }

    return ( retState );
}


/**
 * \brief Stops the transfer of the request and the hardware function: the Gpdma channels are stopped (DMA mode), the
 *        function is stopped and the interrupts and the DMA requests of the unit are disabled. The filter is stopped
 *        (the pipeline is undefined), no request is in progress.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the channels and the function are stopped. Otherwise returns
 *         \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Stop_Transfer( void )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

    const fmac_RequestState_t dmaState  = Fmac_Dma_Stop();
    /* The DMA request enables can be changed only while no function runs */
    const fmac_RequestState_t stopState = Fmac_Reg_Stop_Function();

    Fmac_Reset_RegBit( FMAC_REG_CR, FMAC_CR_RUN_MASK );

    fmac_RuntimeData.FilterState = FMAC_FILTER_STOPPED;
    fmac_RuntimeData.XferState   = FMAC_XFER_STATE_IDLE;
    fmac_RuntimeData.ReqStream   = FMAC_FUNCTION_INACTIVE;

    if( ( FMAC_REQUEST_OK == dmaState  ) &&
        ( FMAC_REQUEST_OK == stopState )    )
    {
        retState = FMAC_REQUEST_OK;
    }
    else
    {
        retState = FMAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes the hardware: resets the unit, clears the control register, configures the interrupt (interrupt
 *        and DMA mode) and the Gpdma channels (DMA mode) and verifies the result by a read-back of the registers.
 *
 * \param irqPrio [in]: Interrupt priority (interrupt and DMA mode).
 *
 * \return Returns \ref FMAC_REQUEST_OK if every step succeeded and the read-back is as required.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Init_Hardware( fmac_IrqPrio_t irqPrio )
{
    fmac_RequestState_t retState = Fmac_Reg_Reset_Unit();

    if( FMAC_REQUEST_OK == retState )
    {
        Fmac_Set_RegVal( FMAC_REG_CR, FMAC_REG_FULL_MASK, FMAC_BITS_CLEAR );
    }
    else
    {
        /* The reset of the unit did not finish */
    }

    if( ( FMAC_REQUEST_OK    == retState                  ) &&
        ( FMAC_XFER_MODE_POLL != fmac_RuntimeData.XferMode )    )
    {
        nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Handler( NVIC_PERIPH_IRQ_FMAC, Fmac_IsrHandler );

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_FMAC, irqPrio );
        }
        else
        {
            /* Handler registration already failed */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Active( NVIC_PERIPH_IRQ_FMAC );
        }
        else
        {
            /* Priority configuration already failed */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = FMAC_REQUEST_OK;
        }
        else
        {
            retState = FMAC_REQUEST_ERROR;
        }
    }
    else
    {
        /* POLL mode does not use the interrupt or the unit was not reset */
    }

    if( ( FMAC_REQUEST_OK    == retState                  ) &&
        ( FMAC_XFER_MODE_DMA == fmac_RuntimeData.XferMode )    )
    {
        retState = Fmac_Dma_Init();
    }
    else
    {
        /* The Gpdma channels are not used or a previous step failed */
    }

    if( FMAC_REQUEST_OK == retState )
    {
        /* Read-back of the registers: control register cleared, parameter register and error flags reset */
        const uint32_t crValue    = Fmac_Get_RegVal( FMAC_REG_CR, FMAC_REG_FULL_MASK );
        const uint32_t paramValue = Fmac_Get_RegVal( FMAC_REG_PARAM, FMAC_REG_FULL_MASK );
        const uint32_t errorFlags = Fmac_Get_RegVal( FMAC_REG_SR, FMAC_SR_ERROR_MASK );

        if( ( FMAC_BITS_CLEAR == crValue    ) &&
            ( FMAC_BITS_CLEAR == paramValue ) &&
            ( FMAC_BITS_CLEAR == errorFlags )    )
        {
            retState = FMAC_REQUEST_OK;
        }
        else
        {
            retState = FMAC_REQUEST_ERROR;
        }
    }
    else
    {
        /* A previous step failed */
    }

    return ( retState );
}


/**
 * \brief Initializes the two Gpdma channels of the DMA mode (the transfers are configured by the requests) and
 *        enables their interrupts in Nvic.
 *
 * \return Returns \ref FMAC_REQUEST_OK if both channels are initialized. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Dma_Init( void )
{
    fmac_RequestState_t retState = FMAC_REQUEST_OK;

    for( fmac_DmaStream_t stream = FMAC_DMA_STREAM_WRITE; FMAC_DMA_STREAM_CNT > stream; stream ++ )
    {
        if( FMAC_REQUEST_OK == retState )
        {
            const gpdma_ChannelId_t channelId = Fmac_Dma_Get_ChannelId( stream );
            gpdma_TransferConfig_t  xferConfig = { 0u };
            gpdma_ConfigStruct_t    dmaConfig  = { 0u };

            Fmac_Dma_Get_XferConfig( stream,
                                     (uint32_t)(uintptr_t)&fmac_DmaDummy,
                                     FMAC_DMA_PLACEHOLDER_CNT,
                                     FMAC_BUFFER_ONE_SHOT,
                                     &xferConfig );

            dmaConfig.PeriphId           = fmac_RuntimeData.DmaPeriphId;
            dmaConfig.ChannelId          = channelId;
            dmaConfig.ChannelPrio        = fmac_RuntimeData.DmaPriority;
            dmaConfig.TransferExecMode   = GPDMA_XFER_EXEC_CONTINUOUS;
            dmaConfig.TransferConfig     = &xferConfig;
            dmaConfig.TransfersCount     = FMAC_DMA_TRANSFERS_CNT;
            dmaConfig.XferListAccessMode = GPDMA_TRANSFER_LIST_ACCESS_SINGLE;
            dmaConfig.XferList           = &fmac_DmaXferList[ stream ];
            dmaConfig.TransferLockState  = GPDMA_TRANSFER_LIST_LOCKED;
            dmaConfig.TransferCompleteIsr = GPDMA_NULL_PTR;
            dmaConfig.HalfTransferIsr     = GPDMA_NULL_PTR;
            dmaConfig.ErrorIsr            = Fmac_Dma_Error;
            dmaConfig.ErrorMask           = (gpdma_ErrorMaskId_t)FMAC_DMA_ERROR_MASK;

            if( FMAC_DMA_STREAM_READ == stream )
            {
                /* The results of the filter end the request: the complete and the half callbacks */
                dmaConfig.TransferCompleteIsr = Fmac_Dma_ReadComplete;

                if( FMAC_NULL_PTR != fmac_RuntimeData.XferHalfCallback )
                {
                    dmaConfig.HalfTransferIsr = Fmac_Dma_ReadHalf;
                }
                else
                {
                    /* No half callback registered */
                }
            }
            else
            {
                /* The write channel reports only its errors */
            }

            gpdma_RequestState_t dmaState = Gpdma_Init( &dmaConfig );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                fmac_RuntimeData.DmaChannelInit[ stream ] = FMAC_FUNCTION_ACTIVE;

                dmaState = Gpdma_Set_InterruptActive( fmac_RuntimeData.DmaPeriphId, channelId );
            }
            else
            {
                /* The channel is not initialized */
            }

            if( GPDMA_REQUEST_OK != dmaState )
            {
                retState = FMAC_REQUEST_ERROR;
            }
            else
            {
                /* The channel is ready */
            }
        }
        else
        {
            /* A previous channel failed */
        }
    }

    return ( retState );
}


/**
 * \brief Releases the Gpdma channels which were initialized by the module.
 *
 * \return Returns \ref FMAC_REQUEST_OK if every initialized channel was released (also if there is none).
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Dma_Deinit( void )
{
    fmac_RequestState_t retState = FMAC_REQUEST_OK;

    for( fmac_DmaStream_t stream = FMAC_DMA_STREAM_WRITE; FMAC_DMA_STREAM_CNT > stream; stream ++ )
    {
        if( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.DmaChannelInit[ stream ] )
        {
            const gpdma_RequestState_t dmaState = Gpdma_Deinit( fmac_RuntimeData.DmaPeriphId,
                                                                Fmac_Dma_Get_ChannelId( stream ) );

            if( GPDMA_REQUEST_OK != dmaState )
            {
                retState = FMAC_REQUEST_ERROR;
            }
            else
            {
                /* The channel is released */
            }
        }
        else
        {
            /* The channel was not initialized by the module */
        }
    }

    return ( retState );
}


/**
 * \brief Stops the Gpdma channels which were initialized by the module.
 *
 * \return Returns \ref FMAC_REQUEST_OK if every initialized channel was stopped (also if there is none).
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Dma_Stop( void )
{
    fmac_RequestState_t retState = FMAC_REQUEST_OK;

    for( fmac_DmaStream_t stream = FMAC_DMA_STREAM_WRITE; FMAC_DMA_STREAM_CNT > stream; stream ++ )
    {
        if( FMAC_FUNCTION_ACTIVE == fmac_RuntimeData.DmaChannelInit[ stream ] )
        {
            const gpdma_RequestState_t dmaState = Gpdma_Set_ChannelInactive( fmac_RuntimeData.DmaPeriphId,
                                                                             Fmac_Dma_Get_ChannelId( stream ) );

            if( GPDMA_REQUEST_OK != dmaState )
            {
                retState = FMAC_REQUEST_ERROR;
            }
            else
            {
                /* The channel is stopped */
            }
        }
        else
        {
            /* The channel was not initialized by the module */
        }
    }

    return ( retState );
}


/**
 * \brief Starts the Gpdma channel of the results (and of the input samples when the request writes some).
 *
 * \return Returns \ref FMAC_REQUEST_OK if the channels are armed. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Dma_Start( void )
{
    fmac_RequestState_t retState = Fmac_Dma_Arm( FMAC_DMA_STREAM_READ,
                                                 (uint32_t)(uintptr_t)fmac_RuntimeData.ReqOutput,
                                                 fmac_RuntimeData.ReqOutputCnt );

    if( ( FMAC_REQUEST_OK == retState                  ) &&
        ( 0u               < fmac_RuntimeData.ReqInputCnt )    )
    {
        retState = Fmac_Dma_Arm( FMAC_DMA_STREAM_WRITE,
                                 (uint32_t)(uintptr_t)fmac_RuntimeData.ReqInput,
                                 fmac_RuntimeData.ReqInputCnt );
    }
    else
    {
        /* A previous step failed or the request writes no sample */
    }

    return ( retState );
}


/**
 * \brief Arms a Gpdma channel for the request: generates the transfer list node (cyclic for a circular stream, once
 *        otherwise), loads it into the channel and enables the channel.
 *
 * \param stream     [in]: Channel of the input samples or of the results.
 * \param memoryAddr [in]: Address of the memory buffer.
 * \param sampleCnt  [in]: Count of the samples of the buffer.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the channel is enabled. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Dma_Arm( fmac_DmaStream_t stream, uint32_t memoryAddr, fmac_SampleCnt_t sampleCnt )
{
    fmac_RequestState_t       retState    = FMAC_REQUEST_ERROR;
    const gpdma_ChannelId_t   channelId   = Fmac_Dma_Get_ChannelId( stream );
    const gpdma_PeriphId_t    periphId    = fmac_RuntimeData.DmaPeriphId;
    const gpdma_ChannelType_t channelType = fmac_DmaChannelTypeLut[ channelId ];
    volatile gpdma_XferList_t *xferNode   = &fmac_DmaXferList[ stream ];
    gpdma_TransferConfig_t    xferConfig  = { 0u };
    gpdma_XferLinkReg_t       linkReg     = 0u;
    gpdma_RequestState_t      dmaState    = GPDMA_REQUEST_ERROR;

    Fmac_Dma_Get_XferConfig( stream, memoryAddr, sampleCnt, fmac_RuntimeData.ReqBufferMode, &xferConfig );

    dmaState = Gpdma_Get_XferListConfig( &xferConfig, FMAC_DMA_TRANSFERS_CNT, channelType, xferNode );

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaState = Gpdma_Get_XferList_LinkReg( xferNode, channelType, &linkReg );
    }
    else
    {
        /* The transfer list node was not generated */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaState = Gpdma_Set_XferListLink( periphId, channelId, linkReg );
    }
    else
    {
        /* The link register is not available */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaState = Gpdma_Set_BlockSize( periphId, channelId, xferConfig.BlockSize );
    }
    else
    {
        /* The link register was not configured */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaState = Gpdma_Set_SourceAddr( periphId, channelId, xferConfig.SourceAddr );
    }
    else
    {
        /* The block size was not configured */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaState = Gpdma_Set_DestinationAddr( periphId, channelId, xferConfig.DestinationAddr );
    }
    else
    {
        /* The source address was not configured */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaState = Gpdma_Set_ChannelActive( periphId, channelId );
    }
    else
    {
        /* The destination address was not configured */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        retState = FMAC_REQUEST_OK;
    }
    else
    {
        retState = FMAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Fills the Gpdma transfer configuration of a channel: the input samples go from the memory to FMAC_WDATA, the
 *        results from FMAC_RDATA to the memory, 16 bit data, one request per sample, one block of the buffer (a
 *        cyclic node in the circular mode).
 *
 * \param stream     [in]:  Channel of the input samples or of the results.
 * \param memoryAddr [in]:  Address of the memory buffer.
 * \param sampleCnt  [in]:  Count of the samples of the buffer.
 * \param bufferMode [in]:  Circular or one shot buffer.
 * \param xferConfig [out]: Pointer to the transfer configuration to fill. Must not be NULL.
 */
static void Fmac_Dma_Get_XferConfig( fmac_DmaStream_t stream,
                                     uint32_t memoryAddr,
                                     fmac_SampleCnt_t sampleCnt,
                                     fmac_BufferMode_t bufferMode,
                                     gpdma_TransferConfig_t * const xferConfig )
{
    if( FMAC_BUFFER_CIRCULAR == bufferMode )
    {
        xferConfig->XferListExecMode = GPDMA_XFER_LIST_EXEC_CYCLIC_ALL;
    }
    else
    {
        xferConfig->XferListExecMode = GPDMA_XFER_LIST_EXEC_ONCE;
    }

    xferConfig->Direction              = fmac_DmaDirectionLut[ stream ];
    xferConfig->EventMode              = GPDMA_TRANSFER_EVENT_BLOCK;
    xferConfig->TriggerType            = GPDMA_TRG_NOT_USED;
    xferConfig->TriggerMode            = GPDMA_TRIGGER_BLOCK;
    xferConfig->RequestSource          = fmac_DmaRequestLut[ stream ];
    xferConfig->RequestMode            = GPDMA_PERIPH_REQ_SINGLE;
    xferConfig->BlockSize              = (gpdma_BlockSize_t)( sampleCnt * (fmac_SampleCnt_t)sizeof( fmac_Sample_t ) );
    xferConfig->BlockRepetitionCount   = 0u;

    xferConfig->SourceDataSize         = GPDMA_DATA_SIZE_16BITS;
    xferConfig->SourceBurstLength      = FMAC_DMA_BURST_LEN;
    xferConfig->SourcePortId           = GPDMA_PORT_DEFAULT;
    xferConfig->SourceDataOp           = GPDMA_SRC_DATA_PRESERVE;

    xferConfig->DestinationDataSize    = GPDMA_DATA_SIZE_16BITS;
    xferConfig->DestinationBurstLength = FMAC_DMA_BURST_LEN;
    xferConfig->DestinationPortId      = GPDMA_PORT_DEFAULT;
    xferConfig->DestinationDataOp      = GPDMA_DEST_DATA_PRESERVE;

    if( FMAC_DMA_STREAM_WRITE == stream )
    {
        xferConfig->SourceAddr          = (gpdma_SrcAddr_t)memoryAddr;
        xferConfig->SourceAddrMode      = GPDMA_ADDR_INCREMENT;
        xferConfig->DestinationAddr     = (gpdma_DstAddr_t)Fmac_Reg_Get_WdataAddr();
        xferConfig->DestinationAddrMode = GPDMA_ADDR_STATIC;
    }
    else
    {
        xferConfig->SourceAddr          = (gpdma_SrcAddr_t)Fmac_Reg_Get_RdataAddr();
        xferConfig->SourceAddrMode      = GPDMA_ADDR_STATIC;
        xferConfig->DestinationAddr     = (gpdma_DstAddr_t)memoryAddr;
        xferConfig->DestinationAddrMode = GPDMA_ADDR_INCREMENT;
    }
}


/**
 * \brief Returns the Gpdma channel of the input samples or of the results.
 *
 * \param stream [in]: Channel of the input samples or of the results.
 *
 * \return Gpdma channel of the configuration of the module.
 */
static gpdma_ChannelId_t Fmac_Dma_Get_ChannelId( fmac_DmaStream_t stream )
{
    gpdma_ChannelId_t retChannelId = fmac_RuntimeData.DmaReadChannelId;

    if( FMAC_DMA_STREAM_WRITE == stream )
    {
        retChannelId = fmac_RuntimeData.DmaWriteChannelId;
    }
    else
    {
        retChannelId = fmac_RuntimeData.DmaReadChannelId;
    }

    return ( retChannelId );
}


/**
 * \brief Releases the peripheral: stops the Gpdma channels and the function, clears the control register, disables
 *        the interrupt, releases the Gpdma channels, resets the unit, disables the clock and clears the runtime data.
 *
 * \return Returns \ref FMAC_REQUEST_OK if every step succeeded. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t Fmac_Release( void )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

    const fmac_RequestState_t dmaStopState = Fmac_Dma_Stop();
    const fmac_RequestState_t stopState    = Fmac_Reg_Stop_Function();

    Fmac_Set_RegVal( FMAC_REG_CR, FMAC_REG_FULL_MASK, FMAC_BITS_CLEAR );

    nvic_RequestState_t nvicState = NVIC_REQUEST_OK;

    if( FMAC_XFER_MODE_POLL != fmac_RuntimeData.XferMode )
    {
        nvicState = Nvic_Set_PeriphIrq_Inactive( NVIC_PERIPH_IRQ_FMAC );
    }
    else
    {
        /* The interrupt is not used */
    }

    const fmac_RequestState_t dmaDeinitState = Fmac_Dma_Deinit();
    const rcc_RequestState_t  rstActState    = Rcc_Set_ResetActive( RCC_PERIPH_FMAC );
    const rcc_RequestState_t  rstInactState  = Rcc_Set_ResetInactive( RCC_PERIPH_FMAC );
    const rcc_RequestState_t  clkState       = Rcc_Set_PeriphInactive( RCC_PERIPH_FMAC );

    Fmac_Reset_Runtime();

    if( ( FMAC_REQUEST_OK == dmaStopState   ) &&
        ( FMAC_REQUEST_OK == stopState      ) &&
        ( NVIC_REQUEST_OK == nvicState      ) &&
        ( FMAC_REQUEST_OK == dmaDeinitState ) &&
        ( RCC_REQUEST_OK  == rstActState    ) &&
        ( RCC_REQUEST_OK  == rstInactState  ) &&
        ( RCC_REQUEST_OK  == clkState       )    )
    {
        retState = FMAC_REQUEST_OK;
    }
    else
    {
        retState = FMAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Clears the runtime data: the module is not initialized, the filter is not configured, no request runs and
 *        no callback is registered.
 */
static void Fmac_Reset_Runtime( void )
{
    fmac_RuntimeData.InitState            = FMAC_FUNCTION_INACTIVE;
    fmac_RuntimeData.XferMode             = FMAC_XFER_MODE_POLL;
    fmac_RuntimeData.XferHalfCallback     = FMAC_NULL_PTR;
    fmac_RuntimeData.XferCompleteCallback = FMAC_NULL_PTR;
    fmac_RuntimeData.ErrorCallback        = FMAC_NULL_PTR;
    fmac_RuntimeData.FilterState          = FMAC_FILTER_UNCONFIGURED;
    fmac_RuntimeData.XferState            = FMAC_XFER_STATE_IDLE;
    fmac_RuntimeData.XferError            = FMAC_XFER_ERROR_NONE;
    fmac_RuntimeData.ErrorBaseline        = FMAC_XFER_ERROR_NONE;
    fmac_RuntimeData.ReqInput             = FMAC_NULL_PTR;
    fmac_RuntimeData.ReqOutput            = FMAC_NULL_PTR;
    fmac_RuntimeData.ReqInputCnt          = 0u;
    fmac_RuntimeData.ReqOutputCnt         = 0u;
    fmac_RuntimeData.ReqInputIdx          = 0u;
    fmac_RuntimeData.ReqOutputIdx         = 0u;
    fmac_RuntimeData.ReqHalfIdx           = 0u;
    fmac_RuntimeData.ReqStream            = FMAC_FUNCTION_INACTIVE;
    fmac_RuntimeData.ReqBufferMode        = FMAC_BUFFER_ONE_SHOT;

    for( fmac_DmaStream_t stream = FMAC_DMA_STREAM_WRITE; FMAC_DMA_STREAM_CNT > stream; stream ++ )
    {
        fmac_RuntimeData.DmaChannelInit[ stream ] = FMAC_FUNCTION_INACTIVE;
    }
}


/**
 * \brief Serves the request in progress (poll and interrupt mode): moves the samples, checks the errors and ends a
 *        block request or a one shot stream when all results are read.
 */
static void Fmac_Service( void )
{
    if( FMAC_XFER_STATE_BUSY == fmac_RuntimeData.XferState )
    {
        Fmac_Feed();
        Fmac_Drain();
        Fmac_Check_Errors();

        const fmac_FunctionState_t circular = Fmac_Is_Circular();

        if( ( FMAC_XFER_STATE_BUSY          == fmac_RuntimeData.XferState    ) &&
            ( FMAC_FUNCTION_INACTIVE        == circular                      ) &&
            ( fmac_RuntimeData.ReqOutputCnt <= fmac_RuntimeData.ReqOutputIdx )    )
        {
            Fmac_Finish_Request();
        }
        else
        {
            /* The request goes on, ended with an error or is a circular stream */
        }
    }
    else
    {
        /* No request is in progress */
    }
}


/**
 * \brief Writes the input samples of the request to the filter while the input buffer has a free place; a circular
 *        stream starts again at the begin of the input buffer. The write interrupt is disabled when the last sample
 *        of a block request or of a one shot stream is written.
 */
static void Fmac_Feed( void )
{
    const fmac_FunctionState_t circular   = Fmac_Is_Circular();
    fmac_SampleCnt_t           pendingCnt = ( fmac_RuntimeData.ReqInputCnt - fmac_RuntimeData.ReqInputIdx );

    if( FMAC_FUNCTION_ACTIVE == circular )
    {
        /* One lap of the buffer at most by one call */
        pendingCnt = fmac_RuntimeData.ReqInputCnt;
    }
    else
    {
        /* The samples which are not written yet */
    }

    for( fmac_SampleCnt_t stepCnt = 0u; pendingCnt > stepCnt; stepCnt ++ )
    {
        const uint32_t fullFlag = Fmac_Get_RegBit( FMAC_REG_SR, FMAC_SR_X1FULL );

        if( FMAC_BITS_CLEAR == fullFlag )
        {
            Fmac_Reg_Write_Sample( fmac_RuntimeData.ReqInput[ fmac_RuntimeData.ReqInputIdx ] );

            fmac_RuntimeData.ReqInputIdx ++;

            if( ( fmac_RuntimeData.ReqInputCnt <= fmac_RuntimeData.ReqInputIdx ) &&
                ( FMAC_FUNCTION_ACTIVE         == circular                     )    )
            {
                fmac_RuntimeData.ReqInputIdx = 0u;
            }
            else
            {
                /* The input buffer is not at its end or the request ends with its samples */
            }
        }
        else
        {
            /* The input buffer is full, the rest is written later */
            break;
        }
    }

    if( ( fmac_RuntimeData.ReqInputCnt <= fmac_RuntimeData.ReqInputIdx ) &&
        ( FMAC_FUNCTION_INACTIVE       == circular                     )    )
    {
        Fmac_Reset_RegBit( FMAC_REG_CR, FMAC_CR_WIEN );
    }
    else
    {
        /* More samples to write */
    }
}


/**
 * \brief Reads the results of the filter into the output buffer of the request while the filter has a result. The
 *        half callback is called when half of the results are read; a circular stream calls the complete callback and
 *        starts again at the begin of the output buffer when the buffer is full.
 */
static void Fmac_Drain( void )
{
    const fmac_FunctionState_t circular   = Fmac_Is_Circular();
    fmac_SampleCnt_t           pendingCnt = ( fmac_RuntimeData.ReqOutputCnt - fmac_RuntimeData.ReqOutputIdx );

    if( FMAC_FUNCTION_ACTIVE == circular )
    {
        /* One lap of the buffer at most by one call */
        pendingCnt = fmac_RuntimeData.ReqOutputCnt;
    }
    else
    {
        /* The results which are not read yet */
    }

    for( fmac_SampleCnt_t stepCnt = 0u; pendingCnt > stepCnt; stepCnt ++ )
    {
        const uint32_t emptyFlag = Fmac_Get_RegBit( FMAC_REG_SR, FMAC_SR_YEMPTY );

        if( FMAC_BITS_CLEAR == emptyFlag )
        {
            fmac_RuntimeData.ReqOutput[ fmac_RuntimeData.ReqOutputIdx ] = Fmac_Reg_Read_Sample();

            fmac_RuntimeData.ReqOutputIdx ++;

            if( ( fmac_RuntimeData.ReqHalfIdx      == fmac_RuntimeData.ReqOutputIdx ) &&
                ( FMAC_NULL_PTR                    != fmac_RuntimeData.XferHalfCallback )    )
            {
                fmac_RuntimeData.XferHalfCallback();
            }
            else
            {
                /* Not the half of the results or no callback registered */
            }

            if( ( fmac_RuntimeData.ReqOutputCnt <= fmac_RuntimeData.ReqOutputIdx ) &&
                ( FMAC_FUNCTION_ACTIVE          == circular                       )    )
            {
                /* End of a lap: the buffer starts again, the callback may stop the stream */
                fmac_RuntimeData.ReqOutputIdx = 0u;

                if( FMAC_NULL_PTR != fmac_RuntimeData.XferCompleteCallback )
                {
                    fmac_RuntimeData.XferCompleteCallback();
                }
                else
                {
                    /* No complete callback registered */
                }
            }
            else
            {
                /* The output buffer is not at its end or the request ends with its results */
            }
        }
        else
        {
            /* The filter has no result, the rest is read later */
            break;
        }

        if( FMAC_XFER_STATE_BUSY != fmac_RuntimeData.XferState )
        {
            /* A callback stopped the request */
            break;
        }
        else
        {
            /* The request goes on */
        }
    }
}


/**
 * \brief Compares the hardware error flags with the errors already known (latched or set before the request) and
 *        reports every new error by \ref Fmac_Report_Errors.
 */
static void Fmac_Check_Errors( void )
{
    const uint32_t hwErrors    = (uint32_t)Fmac_Get_HwErrors();
    const uint32_t knownErrors = ( (uint32_t)fmac_RuntimeData.ErrorBaseline | (uint32_t)fmac_RuntimeData.XferError );
    const uint32_t newErrors   = ( hwErrors & ~knownErrors );

    if( (uint32_t)FMAC_XFER_ERROR_NONE != newErrors )
    {
        Fmac_Report_Errors( (fmac_XferError_t)newErrors );
    }
    else
    {
        /* No new error */
    }
}


/**
 * \brief Latches new errors of the request and calls the error callback. A fatal error (overflow, underflow, DMA)
 *        ends the request, the saturation does not (its interrupt is disabled: the flag stays set until the unit is
 *        reset).
 *
 * \param newErrors [in]: Errors detected since the last report, bit mask of \ref fmac_XferError_t.
 */
static void Fmac_Report_Errors( fmac_XferError_t newErrors )
{
    fmac_RuntimeData.XferError = (fmac_XferError_t)( (uint32_t)fmac_RuntimeData.XferError | (uint32_t)newErrors );

    if( (uint32_t)FMAC_XFER_ERROR_NONE != ( (uint32_t)newErrors & (uint32_t)FMAC_XFER_ERROR_FATAL ) )
    {
        Fmac_Abort_Request();
    }
    else
    {
        /* The saturation does not end the request */
    }

    if( (uint32_t)FMAC_XFER_ERROR_NONE != ( (uint32_t)newErrors & (uint32_t)FMAC_XFER_ERROR_SATURATION ) )
    {
        Fmac_Reset_RegBit( FMAC_REG_CR, FMAC_CR_SATIEN );
    }
    else
    {
        /* No saturation */
    }

    if( FMAC_NULL_PTR != fmac_RuntimeData.ErrorCallback )
    {
        fmac_RuntimeData.ErrorCallback( newErrors );
    }
    else
    {
        /* No error callback registered */
    }
}


/**
 * \brief Ends a block request or a one shot stream after the last result was read: a vector function stops its
 *        hardware function and the DMA requests, the interrupts of the data requests are disabled, the state is idle
 *        again and the complete callback is called.
 */
static void Fmac_Finish_Request( void )
{
    const fmac_FunctionState_t vector = Fmac_Is_VectorFunction();

    if( FMAC_FUNCTION_ACTIVE == vector )
    {
        /* The DMA request enables can be changed only while no function runs */
        (void)Fmac_Reg_Stop_Function();

        Fmac_Reset_RegBit( FMAC_REG_CR, FMAC_CR_DATA_REQ_MASK );
    }
    else
    {
        Fmac_Reset_RegBit( FMAC_REG_CR, FMAC_CR_DATA_IRQ_MASK );
    }

    fmac_RuntimeData.XferState = FMAC_XFER_STATE_IDLE;
    fmac_RuntimeData.ReqStream = FMAC_FUNCTION_INACTIVE;

    if( FMAC_NULL_PTR != fmac_RuntimeData.XferCompleteCallback )
    {
        fmac_RuntimeData.XferCompleteCallback();
    }
    else
    {
        /* No complete callback registered */
    }
}


/**
 * \brief Ends the request in progress after a fatal error: the Gpdma channels (DMA mode) and the function of the
 *        hardware are stopped and the interrupts and the DMA requests of the unit are disabled. The pipeline of the
 *        filter is undefined, a preload is required before the filter is started again.
 */
static void Fmac_Abort_Request( void )
{
    (void)Fmac_Dma_Stop();

    /* The DMA request enables can be changed only while no function runs */
    (void)Fmac_Reg_Stop_Function();

    Fmac_Reset_RegBit( FMAC_REG_CR, FMAC_CR_RUN_MASK );

    fmac_RuntimeData.XferState   = FMAC_XFER_STATE_ERROR;
    fmac_RuntimeData.FilterState = FMAC_FILTER_STOPPED;
    fmac_RuntimeData.ReqStream   = FMAC_FUNCTION_INACTIVE;
}


/**
 * \brief Reads the error flags of FMAC_SR and converts them to the errors of the module.
 *
 * \return Bit mask of \ref fmac_XferError_t of the flags which are set.
 */
static fmac_XferError_t Fmac_Get_HwErrors( void )
{
    uint32_t       retErrors   = (uint32_t)FMAC_XFER_ERROR_NONE;
    const uint32_t statusValue = Fmac_Get_RegVal( FMAC_REG_SR, FMAC_SR_ERROR_MASK );

    if( FMAC_BITS_CLEAR != ( statusValue & FMAC_SR_SAT ) )
    {
        retErrors |= (uint32_t)FMAC_XFER_ERROR_SATURATION;
    }
    else
    {
        /* No saturation */
    }

    if( FMAC_BITS_CLEAR != ( statusValue & FMAC_SR_UNFL ) )
    {
        retErrors |= (uint32_t)FMAC_XFER_ERROR_UNDERFLOW;
    }
    else
    {
        /* No underflow */
    }

    if( FMAC_BITS_CLEAR != ( statusValue & FMAC_SR_OVFL ) )
    {
        retErrors |= (uint32_t)FMAC_XFER_ERROR_OVERFLOW;
    }
    else
    {
        /* No overflow */
    }

    return ( (fmac_XferError_t)retErrors );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/**
 * \brief FMAC interrupt handler: serves the request in progress in interrupt mode, checks the error flags in DMA mode
 *        (the data are moved by the Gpdma channels).
 */
static void Fmac_IsrHandler( void )
{
    if( FMAC_XFER_MODE_DMA == fmac_RuntimeData.XferMode )
    {
        if( FMAC_XFER_STATE_BUSY == fmac_RuntimeData.XferState )
        {
            Fmac_Check_Errors();
        }
        else
        {
            /* No request is in progress */
        }
    }
    else
    {
        Fmac_Service();
    }
}


/**
 * \brief Gpdma transfer complete callback of the results (DMA mode): all results of a block request or of a one shot
 *        stream are in the output buffer, the request ends; a circular stream calls the complete callback at the end
 *        of every lap.
 */
static void Fmac_Dma_ReadComplete( void )
{
    if( FMAC_XFER_STATE_BUSY == fmac_RuntimeData.XferState )
    {
        const fmac_FunctionState_t circular = Fmac_Is_Circular();

        if( FMAC_FUNCTION_ACTIVE == circular )
        {
            if( FMAC_NULL_PTR != fmac_RuntimeData.XferCompleteCallback )
            {
                fmac_RuntimeData.XferCompleteCallback();
            }
            else
            {
                /* No complete callback registered */
            }
        }
        else
        {
            fmac_RuntimeData.ReqOutputIdx = fmac_RuntimeData.ReqOutputCnt;

            Fmac_Finish_Request();
        }
    }
    else
    {
        /* No request is in progress */
    }
}


/**
 * \brief Gpdma half transfer callback of the results (DMA mode): half of the output buffer is filled.
 */
static void Fmac_Dma_ReadHalf( void )
{
    if( ( FMAC_XFER_STATE_BUSY == fmac_RuntimeData.XferState ) &&
        ( 0u                    < fmac_RuntimeData.ReqHalfIdx ) &&
        ( FMAC_NULL_PTR        != fmac_RuntimeData.XferHalfCallback )    )
    {
        fmac_RuntimeData.XferHalfCallback();
    }
    else
    {
        /* No request in progress, the buffer has no half or no callback registered */
    }
}


/**
 * \brief Gpdma error callback of both channels (DMA mode): the request ends with \ref FMAC_XFER_ERROR_DMA.
 *
 * \param errorMask [in]: Gpdma errors, not used (every Gpdma error is a transfer error of the request).
 */
static void Fmac_Dma_Error( gpdma_ErrorMaskId_t errorMask )
{
    (void)errorMask;

    if( FMAC_XFER_STATE_BUSY == fmac_RuntimeData.XferState )
    {
        Fmac_Report_Errors( FMAC_XFER_ERROR_DMA );
    }
    else
    {
        /* No request is in progress */
    }
}

#endif /* FMAC */

/* ================================ TASKS =================================== */
