/**
 * \author Mr.Nobody
 * \file Test_Fmac.c
 * \ingroup Fmac
 * \brief Unit tests of the filter math accelerator (FMAC) module.
 *
 * Fmac.c and Fmac_Reg.c are compiled unchanged, the FMAC registers are emulated by RegMem, the RCC, NVIC and
 * Dma modules are mocked by CMock.
 *
 * \note Limitations of the register emulation (RegMem registers are plain memory). The tests activate the HW
 *       model UtFmac_HwModel (see RegMem_Set_ModelActive() of the RegMem framework), which emulates
 *       - the reset of the unit (FMAC_CR.RESET is cleared, FMAC_PARAM and the error flags are reset),
 *       - the completion of a load function: FMAC_PARAM.START is cleared after the number of writes to
 *         FMAC_WDATA the function takes (a load function written with fewer values never completes),
 *       - the data written to FMAC_WDATA: every write is recorded (the register is plain memory, the model
 *         empties it after every store so that two equal values are two writes) in the log of the load
 *         function X1 / X2 / Y or in the log of the running filter function,
 *       - the flag FMAC_SR.X1FULL after a number of samples written to the running filter.
 *       The FMAC does not compute: the results are whatever the test presets into FMAC_RDATA (one value for
 *       every read), FMAC_SR.YEMPTY / X1FULL / the error flags are preset by the tests. The tests prove the
 *       register programming, the sequence and the content of the data written to the accelerator, the flow
 *       control by the flags, the callbacks and the states, not the filtered values (see the integration tests).
 *       Every test case is executed in its own process (CTest), the runtime data of the module start from the
 *       reset state in every test.
 *
 * \note The Dma functions the module calls are replaced by stubs (see UtFmac_DmaStubs) which record every call
 *       (function, controller, stream and the value the test checks: transfer mode, memory address, count) in a log,
 *       keep a copy of the configuration of both streams (so that the tests can call the callbacks registered by the
 *       module) and let a chosen call fail. The tests prove which transfers the module arms and how it reacts to the
 *       callbacks; the transfers themselves are executed by the integration tests on hardware.
 *
 * \note The FMAC exists on STM32H723, H725, H730, H733 and H735 only (not on the other STM32H7 lines and not on
 *       STM32H7R / STM32H7S). The test runner is generated from this file without running the preprocessor, so every
 *       test case function below is defined unconditionally (the same list on every line); on the lines without the
 *       peripheral the test cases use
 *       TEST_IGNORE_MESSAGE and the test case of the missing peripheral checks that every function returns an
 *       error.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Fmac_Port.h"                      /* Module under test              */
#include "Fmac.h"                           /* Private interface (test only)  */
#include "Fmac_Reg.h"                       /* Register access (test only)    */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "Stm32.h"                          /* FMAC registers definition      */
#include "stdint.h"                         /* Standard integer types         */
/* ============================= TYPEDEFS =================================== */

#if defined(FMAC)

/** Record of one function started by a write to FMAC_PARAM with the START bit */
typedef struct
{
    uint32_t Func;   /**< FMAC_PARAM.FUNC */
    uint32_t ParamP; /**< FMAC_PARAM.P    */
    uint32_t ParamQ; /**< FMAC_PARAM.Q    */
    uint32_t ParamR; /**< FMAC_PARAM.R    */
}   utFmac_StartRecord_t;

/** Dma functions recorded by the stubs */
typedef enum
{
    UT_FMAC_DMA_INIT = 0u,      /**< Dma_Init                          */
    UT_FMAC_DMA_TE_IRQ,         /**< Dma_Set_TransferErrorIrqActive    */
    UT_FMAC_DMA_TC_IRQ,         /**< Dma_Set_TransferCompleteIrqActive */
    UT_FMAC_DMA_HT_IRQ,         /**< Dma_Set_HalfTransferIrqActive     */
    UT_FMAC_DMA_IRQ_ACTIVE,     /**< Dma_Set_InterruptActive           */
    UT_FMAC_DMA_IRQ_INACTIVE,   /**< Dma_Set_InterruptInactive         */
    UT_FMAC_DMA_MODE,           /**< Dma_Set_TransferMode              */
    UT_FMAC_DMA_MEM_ADDR,       /**< Dma_Set_MemoryAddr                */
    UT_FMAC_DMA_COUNT,          /**< Dma_Set_DataCount                 */
    UT_FMAC_DMA_XFER_ACTIVE,    /**< Dma_Set_TransferActive            */
    UT_FMAC_DMA_XFER_INACTIVE,  /**< Dma_Set_TransferInactive          */
    UT_FMAC_DMA_NONE,           /**< No call (no failure injected)     */
    UT_FMAC_DMA_CNT             /**< Count of the items                */
}   utFmac_DmaCall_t;

/** Record of one Dma call */
typedef struct
{
    utFmac_DmaCall_t Id;        /**< Dma function                          */
    uint32_t         Periph;    /**< Controller of the call                */
    uint32_t         Channel;   /**< Stream of the call                    */
    uint32_t         Value;     /**< Value of the call the test checks     */
}   utFmac_DmaRecord_t;

/** Dma call which fails in a test of a failing arming: the function and the number of its call */
typedef struct
{
    utFmac_DmaCall_t Id;        /**< Dma function                          */
    uint32_t         Nth;       /**< Number of the call which fails        */
}   utFmac_ArmFail_t;

#endif /* FMAC */

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(FMAC)

static void                UtFmac_HwModel          ( void );
static void                UtFmac_ModelLogSample   ( fmac_Sample_t sample );
static void                UtFmac_ModelStart       ( void );
static nvic_RequestState_t UtFmac_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId,
                                                      const nvic_IsrCallback_t irqHandler,
                                                      int callCnt );
static void                UtFmac_XferHalfCallback ( void );
static void                UtFmac_XferCompleteCallback( void );
static void                UtFmac_ErrorCallback    ( fmac_XferError_t errorMask );
static fmac_Config_t       UtFmac_Get_Config       ( fmac_XferMode_t xferMode );
static void                UtFmac_Expect_Init      ( fmac_XferMode_t xferMode );
static void                UtFmac_Expect_Release   ( fmac_XferMode_t xferMode );
static void                UtFmac_Init             ( fmac_XferMode_t xferMode );
static fmac_FilterConfig_t UtFmac_Get_FirConfig    ( void );
static fmac_FilterConfig_t UtFmac_Get_IirConfig    ( void );
static void                UtFmac_Config           ( const fmac_FilterConfig_t * const filterConfig );
static void                UtFmac_Ready            ( const fmac_FilterConfig_t * const filterConfig );
static void                UtFmac_Running          ( const fmac_FilterConfig_t * const filterConfig );
static uint32_t            UtFmac_Get_BufCfg       ( uint32_t base, uint32_t size, uint32_t watermark );
static uint32_t            UtFmac_Get_Param        ( uint32_t func, uint32_t paramP, uint32_t paramQ, uint32_t paramR );
static void                UtFmac_FillOutput       ( void );
static void                UtFmac_AssertOutput     ( fmac_SampleCnt_t resultCnt );
static fmac_RequestState_t UtFmac_StartBlock       ( void );
static void                UtFmac_AssertBufCfg     ( uint32_t regValue,
                                                     uint32_t base,
                                                     uint32_t size,
                                                     uint32_t watermark );
static void                UtFmac_ModelAppend      ( fmac_Sample_t * const logData,
                                                     uint32_t * const logCnt,
                                                     fmac_Sample_t sample );
static dma_RequestState_t  UtFmac_DmaRecord       ( utFmac_DmaCall_t callId,
                                                     uint32_t periphId,
                                                     uint32_t channelId,
                                                     uint32_t value );
static void                UtFmac_DmaStubs         ( void );
static void                UtFmac_ResetDmaLog      ( void );
static void                UtFmac_AssertDma        ( uint32_t logIdx,
                                                     utFmac_DmaCall_t callId,
                                                     uint32_t periphId,
                                                     uint32_t channelId,
                                                     uint32_t value );
static void                UtFmac_AssertArm        ( uint32_t firstIdx,
                                                     dma_ChannelId_t channelId,
                                                     dma_TransferMode_t xferMode,
                                                     uint32_t memoryAddr,
                                                     uint32_t sampleCnt );
static void                UtFmac_AssertInitStream ( uint32_t slot,
                                                     dma_ChannelId_t channelId,
                                                     dma_Direction_t direction,
                                                     dma_PeriphReqId_t request,
                                                     uint32_t periphAddr );
static void                UtFmac_InitDma          ( void );
static fmac_StreamRequest_t UtFmac_Get_StreamRequest( fmac_BufferMode_t bufferMode );
static fmac_RequestState_t UtFmac_StartStream      ( fmac_BufferMode_t bufferMode );
static fmac_RequestState_t UtFmac_StartDotProduct  ( void );
static fmac_FilterConfig_t UtFmac_Get_VectorConfig ( fmac_Function_t function );
static void                UtFmac_AssertVectorRequests( fmac_Function_t function );

#endif /* FMAC */

/* ========================== SYMBOLIC CONSTANTS ============================= */

/** Message of the test cases on a device line without the peripheral */
#define UT_FMAC_NO_PERIPH_MSG               "No FMAC on this device line"

/** Expected version of the module */
#define UT_FMAC_VERSION_MAJOR               ( 0u )
#define UT_FMAC_VERSION_MINOR               ( 1u )
#define UT_FMAC_VERSION_PATCH               ( 0u )

/** Register value without any bit set */
#define UT_FMAC_REG_ZERO                    ( 0u )

/** Interrupt priority used by the tests */
#define UT_FMAC_IRQ_PRIO                    ( 5u )

/** Raw value of FMAC_WDATA after the model consumed a write (bit 31 is reserved, the module never writes it) */
#define UT_FMAC_WDATA_EMPTY                 ( 0x80000000u )

/** Size of the logs of the HW model in samples */
#define UT_FMAC_LOG_SIZE                    ( 600u )

/** Size of the log of the started functions */
#define UT_FMAC_START_LOG_SIZE              ( 16u )

/** Size of the result buffer of the tests and the guard samples behind it */
#define UT_FMAC_OUTPUT_SIZE                 ( 16u )
#define UT_FMAC_GUARD_SIZE                  ( 2u )
#define UT_FMAC_OUTPUT_MEM_SIZE             ( UT_FMAC_GUARD_SIZE + UT_FMAC_OUTPUT_SIZE + UT_FMAC_GUARD_SIZE )

/** Fill pattern of the result buffer (not a result of the tests) and the raw result the tests preset to FMAC_RDATA */
#define UT_FMAC_FILL_PATTERN                ( 0x5A5A )
#define UT_FMAC_RESULT_RAW                  ( 0xFEDCu )
#define UT_FMAC_RESULT                      ( (fmac_Sample_t)0xFEDC )

/** Coefficient counts of the test filters */
#define UT_FMAC_TAPS                        ( 4u )
#define UT_FMAC_FEEDBACK                    ( 2u )

/** Input samples of the test requests */
#define UT_FMAC_INPUT_CNT                   ( 6u )
#define UT_FMAC_INPUT_HALF                  ( 3u )

/** Layout of the FIR test filter (4 coefficients, watermark 1: headroom 1): X2, X1 and Y base and size, words */
#define UT_FMAC_FIR_X2_BASE                 ( 0u )
#define UT_FMAC_FIR_X2_SIZE                 ( 4u )
#define UT_FMAC_FIR_X1_BASE                 ( 4u )
#define UT_FMAC_FIR_X1_SIZE                 ( 5u )
#define UT_FMAC_FIR_Y_BASE                  ( 9u )
#define UT_FMAC_FIR_Y_SIZE                  ( 1u )
#define UT_FMAC_FIR_WORDS                   ( 10u )

/** Layout of the IIR test filter (4 feedforward and 2 feedback coefficients, watermark 1) */
#define UT_FMAC_IIR_X2_SIZE                 ( 6u )
#define UT_FMAC_IIR_X1_BASE                 ( 6u )
#define UT_FMAC_IIR_X1_SIZE                 ( 5u )
#define UT_FMAC_IIR_Y_BASE                  ( 11u )
#define UT_FMAC_IIR_Y_SIZE                  ( 3u )
#define UT_FMAC_IIR_WORDS                   ( 14u )

/** Limits of the hardware used by the boundary tests */
#define UT_FMAC_FIR_MAX_X1_BASE             ( 127u )
#define UT_FMAC_FIR_MAX_X1_SIZE             ( 128u )
#define UT_FMAC_FIR_MAX_Y_BASE              ( 255u )
#define UT_FMAC_IIR_MAX_B                   ( 64u )
#define UT_FMAC_IIR_MAX_A                   ( 63u )
#define UT_FMAC_IIR_MAX_X1_SIZE             ( 65u )
#define UT_FMAC_IIR_MAX_Y_SIZE              ( 64u )
#define UT_FMAC_COEF_ARRAY_SIZE             ( 130u )
#define UT_FMAC_TOO_MANY_COEF               ( 128u )
#define UT_FMAC_TOO_MANY_IIR_B              ( 65u )
#define UT_FMAC_TOO_MANY_IIR_A              ( 64u )
#define UT_FMAC_HEADROOM_OVER               ( 2u )
#define UT_FMAC_HEADROOM_HUGE               ( 257u )
#define UT_FMAC_HEADROOM_SOME               ( 5u )
#define UT_FMAC_MEMORY_OVER                 ( 257u )

/** Watermark encodings (FMAC_X1BUFCFG.FULL_WM / FMAC_YBUFCFG.EMPTY_WM) and thresholds */
#define UT_FMAC_WM_4_CODE                   ( 2u )
#define UT_FMAC_WM_8_CODE                   ( 3u )
#define UT_FMAC_WM_4_THRESHOLD              ( 4u )
#define UT_FMAC_WM_8_THRESHOLD              ( 8u )

/** Layout of the FIR test filter with the input watermark 4 (X1 size 8) and the output watermark 8 (Y at 12) */
#define UT_FMAC_WM_X1_SIZE                  ( 8u )
#define UT_FMAC_WM_Y_BASE                   ( 12u )

/** X1 size of the same filter with a requested input headroom of 9 words (above the threshold of 4) */
#define UT_FMAC_WM_HR_X1_SIZE               ( 13u )

/** Words of the FIR test filter with both watermarks 8 (4 + 12 + 8) and with the input headroom of 5 (4 + 9 + 8) */
#define UT_FMAC_FIR_WM8_WORDS               ( 24u )
#define UT_FMAC_FIR_HEADROOM_WORDS          ( 21u )

/** Y base of the largest IIR (X2 127 words and X1 65 words in front of it) */
#define UT_FMAC_IIR_MAX_Y_BASE              ( 192u )

/** Number of samples written to the running filter after which the model sets FMAC_SR.X1FULL */
#define UT_FMAC_FULL_AFTER                  ( 2u )

/** Histories of the preload tests */
#define UT_FMAC_HISTORY_IN_CNT              ( 2u )
#define UT_FMAC_HISTORY_OUT_CNT             ( 1u )
#define UT_FMAC_HISTORY_ZERO_CNT            ( 1u )

/** Count of the load functions and number of the loads of one configuration */
#define UT_FMAC_RESETS_CONFIG               ( 2u )
#define UT_FMAC_RESETS_PRELOAD              ( 3u )
#define UT_FMAC_LOADS_FIR                   ( 2u )
#define UT_FMAC_LOADS_IIR                   ( 3u )
#define UT_FMAC_LOAD_X2_IDX                 ( 0u )
#define UT_FMAC_LOAD_X1_IDX                 ( 1u )
#define UT_FMAC_LOAD_Y_IDX                  ( 2u )

/** Floating point conversion test values and their q1.15 samples */
#define UT_FMAC_FLOAT_HALF                  ( 0.5f )
#define UT_FMAC_FLOAT_HALF_NEG              ( -0.5f )
#define UT_FMAC_FLOAT_ONE                   ( 1.0f )
#define UT_FMAC_FLOAT_ONE_NEG               ( -1.0f )
#define UT_FMAC_FLOAT_TWO                   ( 2.0f )
#define UT_FMAC_FLOAT_TWO_NEG               ( -2.0f )
#define UT_FMAC_FLOAT_LARGEST               ( 0.99996948f )
#define UT_FMAC_FLOAT_LSB_FRACTION_LOW      ( 0.4f )
#define UT_FMAC_FLOAT_LSB_FRACTION_HIGH     ( 0.6f )
#define UT_FMAC_FLOAT_LSB                   ( 0.000030517578125f )
#define UT_FMAC_FLOAT_ZERO                  ( 0.0f )
#define UT_FMAC_SAMPLE_HALF                 ( (fmac_Sample_t)16384 )
#define UT_FMAC_SAMPLE_MAX                  ( (fmac_Sample_t)32767 )
#define UT_FMAC_SAMPLE_MIN                  ( (fmac_Sample_t)-32768 )
#define UT_FMAC_SAMPLE_ONE_LSB              ( (fmac_Sample_t)1 )

/** Count of two events (calls, errors, streams stopped) */
#define UT_FMAC_TWICE                       ( 2u )

/** Count of the laps of a stream and of the repeated requests of a test */
#define UT_FMAC_STREAM_LAPS                 ( 2u )

/** Count of the Dma streams of the module and size of the log of the Dma calls */
#define UT_FMAC_DMA_STREAMS                 ( 2u )
#define UT_FMAC_DMA_LOG_SIZE                ( 96u )

/** Address of a buffer as the Dma takes it */
#define UT_FMAC_ADDR( ptr )                 ( (uint32_t)(uintptr_t)( ptr ) )

/** Count of the Dma calls which arm a stream and index of the count call of the first stream */
#define UT_FMAC_ARM_CALLS                   ( 4u )
#define UT_FMAC_COUNT_IDX                   ( 2u )

/** Count of the Dma calls of the initialization (the stream of the input data: init, transfer error interrupt,
 *  interrupt line; the stream of the results also the transfer complete and the half transfer interrupt), of the
 *  initialization which fails at the init of the second stream and of the deinitialization (stop of both streams,
 *  release of both streams: stop and interrupt line) */
#define UT_FMAC_INIT_DMA_CALLS              ( 8u )
#define UT_FMAC_INIT_FAIL_DMA_CALLS         ( 7u )
#define UT_FMAC_DEINIT_DMA_CALLS            ( 6u )

/** Count of the Dma calls which initialize the stream of the input data (init, transfer error interrupt, line) */
#define UT_FMAC_INIT_WRITE_CALLS            ( 3u )

/** Count of the samples of the placeholder transfer of the initialization, of the results of a dot product */
#define UT_FMAC_PLACEHOLDER_CNT             ( 1u )
#define UT_FMAC_DOT_RESULT_CNT              ( 1u )

/** Count of the arming steps tested to fail */
#define UT_FMAC_ARM_FAIL_CNT                ( 5u )

/** Functions started by a vector request (X2 load, X1 load, filter function) and the index of the filter function */
#define UT_FMAC_VECTOR_STARTS               ( 3u )
#define UT_FMAC_RUN_IDX                     ( 2u )

/** Error interrupt enables of FMAC_CR */
#define UT_FMAC_CR_ERROR_IRQS               ( FMAC_CR_SATIEN | FMAC_CR_UNFLIEN | FMAC_CR_OVFLIEN )

/** Coefficients and inputs of the test filters (one declaration per line: the checkers read them line by line) */
static const fmac_Sample_t utFmac_CoefB[ UT_FMAC_COEF_ARRAY_SIZE ] = { 1000, -2000, 3000, -4000, 5000, 6000 };
static const fmac_Sample_t utFmac_CoefA[ UT_FMAC_COEF_ARRAY_SIZE ] = { 700, -800, 900 };
static const fmac_Sample_t utFmac_CoefBRev[ UT_FMAC_TAPS ] = { -4000, 3000, -2000, 1000 };
static const fmac_Sample_t utFmac_ZeroState[ UT_FMAC_TAPS ] = { 0, 0, 0, 0 };
static const fmac_Sample_t utFmac_Input[ UT_FMAC_INPUT_CNT ] = { 100, -200, 300, -400, 500, -600 };
static const fmac_Sample_t utFmac_HistoryIn[ UT_FMAC_HISTORY_IN_CNT ] = { 11, -12 };
static const fmac_Sample_t utFmac_HistoryOut[ UT_FMAC_HISTORY_OUT_CNT ] = { 21 };

/* ========================== LOCAL VARIABLES ================================ */

#if defined(FMAC)

/** ISR registered by the module in Nvic (captured by \ref UtFmac_NvicSetHandlerStub) */
static nvic_IsrCallback_t utFmac_Isr;

/** Result of the stub of Nvic_Set_PeriphIrq_Handler */
static nvic_RequestState_t utFmac_NvicHandlerResult;

/** Count of the resets of the unit seen by the HW model */
static uint32_t utFmac_ResetCnt;

/** Value of FMAC_PARAM seen by the HW model at the previous call (detection of a start) */
static uint32_t utFmac_LastParam;

/** Function of the load function in progress and the count of the writes it still takes */
static uint32_t utFmac_LoadFunc;
static uint32_t utFmac_LoadRemaining;

/** The model never completes a load function (START stays set) */
static fmac_FunctionState_t utFmac_NoLoadDone;

/** The model clears START of a filter function at once (a request read out by the interrupt or by the DMA ends) */
static fmac_FunctionState_t utFmac_StartAutoDone;

/** Error flags which stay set in FMAC_SR after a reset (a stuck flag) */
static uint32_t utFmac_StuckSr;

/** Count of the samples written to the running filter after which the model sets X1FULL, 0 for never */
static uint32_t utFmac_X1FullAfter;

/** Count of the functions started and their records */
static uint32_t             utFmac_StartCnt;
static utFmac_StartRecord_t utFmac_StartLog[ UT_FMAC_START_LOG_SIZE ];

/** Samples written to X1 / X2 / Y by the load functions and to the running filter, and their counts */
static fmac_Sample_t utFmac_X1Data[ UT_FMAC_LOG_SIZE ];
static fmac_Sample_t utFmac_X2Data[ UT_FMAC_LOG_SIZE ];
static fmac_Sample_t utFmac_YData[ UT_FMAC_LOG_SIZE ];
static fmac_Sample_t utFmac_RunData[ UT_FMAC_LOG_SIZE ];
static uint32_t      utFmac_X1Cnt;
static uint32_t      utFmac_X2Cnt;
static uint32_t      utFmac_YCnt;
static uint32_t      utFmac_RunCnt;

/** Count of the samples written to FMAC_WDATA while no function took them */
static uint32_t utFmac_StrayCnt;

/** Counters and arguments of the callbacks */
static uint32_t         utFmac_HalfCnt;
static uint32_t         utFmac_CompleteCnt;
static uint32_t         utFmac_ErrorCnt;
static fmac_XferError_t utFmac_LastErrorMask;

/** Result buffer: guard samples, the results and guard samples again */
static fmac_Sample_t utFmac_OutputMem[ UT_FMAC_OUTPUT_MEM_SIZE ];

/** The complete callback of the tests stops the stream */
static fmac_FunctionState_t utFmac_StopInComplete;

/** Log of the Dma calls, their count and the count per function */
static utFmac_DmaRecord_t utFmac_DmaLog[ UT_FMAC_DMA_LOG_SIZE ];
static uint32_t           utFmac_DmaCnt;
static uint32_t           utFmac_DmaIdCnt[ UT_FMAC_DMA_CNT ];

/** Dma function and number of its call which fails (number 0: none) */
static utFmac_DmaCall_t utFmac_DmaFailId;
static uint32_t         utFmac_DmaFailNth;

/** Copies of the configurations passed to Dma_Init (input data, results) */
static dma_ConfigStruct_t utFmac_DmaInitConfig[ UT_FMAC_DMA_STREAMS ];

/** Steps of the arming of a stream which fail in turn: the Dma function and the number of its call */
static const utFmac_ArmFail_t utFmac_ArmFailLut[ UT_FMAC_ARM_FAIL_CNT ] =
{
    { UT_FMAC_DMA_MODE,        1u },
    { UT_FMAC_DMA_MEM_ADDR,    1u },
    { UT_FMAC_DMA_COUNT,       1u },
    { UT_FMAC_DMA_XFER_ACTIVE, 1u },
    { UT_FMAC_DMA_XFER_ACTIVE, UT_FMAC_TWICE },
};

#endif /* FMAC */

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

#if defined(FMAC)
    utFmac_Isr               = NULL;
    utFmac_NvicHandlerResult = NVIC_REQUEST_OK;
    utFmac_ResetCnt          = 0u;
    utFmac_LastParam         = 0u;
    utFmac_LoadFunc          = 0u;
    utFmac_LoadRemaining     = 0u;
    utFmac_NoLoadDone        = FMAC_FUNCTION_INACTIVE;
    utFmac_StartAutoDone     = FMAC_FUNCTION_INACTIVE;
    utFmac_StuckSr           = 0u;
    utFmac_X1FullAfter       = 0u;
    utFmac_StartCnt          = 0u;
    utFmac_X1Cnt             = 0u;
    utFmac_X2Cnt             = 0u;
    utFmac_YCnt              = 0u;
    utFmac_RunCnt            = 0u;
    utFmac_StrayCnt          = 0u;
    utFmac_HalfCnt           = 0u;
    utFmac_CompleteCnt       = 0u;
    utFmac_ErrorCnt          = 0u;
    utFmac_LastErrorMask     = FMAC_XFER_ERROR_NONE;
    utFmac_StopInComplete    = FMAC_FUNCTION_INACTIVE;
    utFmac_DmaFailId         = UT_FMAC_DMA_NONE;
    utFmac_DmaFailNth        = 0u;

    UtFmac_ResetDmaLog();
    UtFmac_DmaStubs();
    Nvic_Set_PeriphIrq_Handler_Stub( UtFmac_NvicSetHandlerStub );
#endif /* FMAC */
}


void tearDown( void )
{
    ( void )RegMem_Set_ModelInactive();
}

/* ======================= MODULE VERSION AND DEFAULTS ====================== */

/**
 * \brief   Fmac_Get_ModuleVersion() returns version of the module.
 *
 * \par Expected results
 * - Version is 0.1.0 (Major 0, Minor 1, Patch 0), also on a device line without the peripheral.
 */
void Ut_Fmac_Get_ModuleVersion_ReturnsVersion( void )
{
    fmac_ModuleVersion_t version = Fmac_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( UT_FMAC_VERSION_MAJOR, version.Major );
    TEST_ASSERT_EQUAL_UINT8( UT_FMAC_VERSION_MINOR, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( UT_FMAC_VERSION_PATCH, version.Patch );
}


/**
 * \brief   Fmac_Get_DefaultConfig() fills the default configuration, rejects NULL.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, POLL transfer mode, DMA1 stream 0 (input data) and stream 1 (results) low priority,
 *   IRQ priority 0, no callbacks.
 * - NULL pointer: FMAC_REQUEST_ERROR.
 */
void Ut_Fmac_Get_DefaultConfig_ReturnsDefaults( void )
{
#if defined(FMAC)
    fmac_Config_t config;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( FMAC_XFER_MODE_POLL, config.XferMode );
    TEST_ASSERT_EQUAL( DMA_PERIPH_1, config.DmaPeriphId );
    TEST_ASSERT_EQUAL( DMA_STREAM_0, config.DmaWriteChannelId );
    TEST_ASSERT_EQUAL( DMA_STREAM_1, config.DmaReadChannelId );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_LOW, config.DmaPriority );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_REG_ZERO, config.IrqPriority );
    TEST_ASSERT_NULL( config.XferHalfCallback );
    TEST_ASSERT_NULL( config.XferCompleteCallback );
    TEST_ASSERT_NULL( config.ErrorCallback );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_DefaultConfig( NULL ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   On a device line without the FMAC every function checks the peripheral and returns an error.
 *
 * \par Expected results
 * - Every public function returns FMAC_REQUEST_ERROR, no pointer is written (the version is still returned).
 * - On a line with the FMAC the test case is ignored.
 */
void Ut_Fmac_NoPeripheral_AllFunctionsReturnError( void )
{
#if !defined(FMAC)
    fmac_Config_t        config;
    fmac_FilterConfig_t  filterConfig  = { .Function = FMAC_FUNC_FIR, .B = utFmac_CoefB, .BCount = UT_FMAC_TAPS };
    fmac_StreamRequest_t streamRequest = { .Input = utFmac_Input, .InputSize = UT_FMAC_INPUT_CNT };
    fmac_Sample_t        result[ UT_FMAC_INPUT_CNT ] = { 0 };
    fmac_SampleCnt_t     sampleCnt     = UT_FMAC_INPUT_CNT;
    fmac_Sample_t        sample        = 0;
    float                value         = UT_FMAC_FLOAT_ZERO;
    fmac_SampleCnt_t     wordCnt       = 0u;
    fmac_FilterState_t   filterState   = FMAC_FILTER_READY;
    fmac_XferState_t     xferState     = FMAC_XFER_STATE_BUSY;
    fmac_XferError_t     xferError     = FMAC_XFER_ERROR_ALL;
    fmac_IrqPrio_t       irqPrio       = UT_FMAC_IRQ_PRIO;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Deinit() );
    Fmac_Task();
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &filterConfig, &wordCnt ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( &filterConfig ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStart() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterProcess( utFmac_Input, sampleCnt, result, sampleCnt ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStart( &streamRequest ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_XferError( &xferError ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FromFloat( UT_FMAC_FLOAT_HALF, &sample ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_ToFloat( UT_FMAC_SAMPLE_HALF, &value ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_IrqPriority( UT_FMAC_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_IrqPriority( &irqPrio ) );

    TEST_ASSERT_EQUAL( FMAC_FILTER_READY, filterState );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_ALL, xferError );
    TEST_ASSERT_EQUAL_UINT32( 0u, wordCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_IRQ_PRIO, irqPrio );
    TEST_ASSERT_EQUAL( 0, sample );
    TEST_ASSERT_EQUAL_FLOAT( UT_FMAC_FLOAT_ZERO, value );
#else
    TEST_IGNORE_MESSAGE( "The FMAC exists on this device line, the missing peripheral is not testable" );
#endif /* !FMAC */
}

/* ============================== INIT / DEINIT ============================== */

/**
 * \brief   Fmac_Init() in POLL mode enables the clock, resets the unit and verifies the registers.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, the clock is enabled through Rcc, the unit is reset once, FMAC_CR is zero, no Nvic call.
 * - The filter state is FMAC_FILTER_UNCONFIGURED, the transfer state FMAC_XFER_STATE_IDLE, no error.
 */
void Ut_Fmac_Init_Poll_ResetsUnitAndVerifies( void )
{
#if defined(FMAC)
    fmac_FilterState_t filterState = FMAC_FILTER_RUNNING;
    fmac_XferState_t   xferState   = FMAC_XFER_STATE_BUSY;
    fmac_XferError_t   xferError   = FMAC_XFER_ERROR_ALL;

    FMAC->CR = FMAC_CR_CLIPEN;
    UtFmac_Init( FMAC_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ResetCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->PARAM );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_UNCONFIGURED, filterState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferError( &xferError ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_NONE, xferError );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() in interrupt mode registers the handler and enables the interrupt in Nvic.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, the handler is registered, the priority of the configuration is set, the interrupt is enabled.
 * - The unit is reset, FMAC_CR is zero (the interrupts of the unit are enabled by the filter start).
 */
void Ut_Fmac_Init_Isr_ConfiguresNvic( void )
{
#if defined(FMAC)
    UtFmac_Init( FMAC_XFER_MODE_ISR );

    TEST_ASSERT_NOT_NULL( utFmac_Isr );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ResetCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->CR );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() rejects a NULL configuration, an invalid transfer mode and a second initialization.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR without any Rcc / Nvic call and without a register write (the unit is not reset).
 * - After a successful initialization a second Fmac_Init() returns FMAC_REQUEST_ERROR and changes nothing.
 */
void Ut_Fmac_Init_InvalidParams_ReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t config = UtFmac_Get_Config( FMAC_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( NULL ) );

    config.XferMode = FMAC_XFER_MODE_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_ResetCnt );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    config = UtFmac_Get_Config( FMAC_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ResetCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() returns an error when Rcc can not enable the clock.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR, no register is written (the unit is not reset), the module stays not initialized.
 */
void Ut_Fmac_Init_RccFails_ReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t      config      = UtFmac_Get_Config( FMAC_XFER_MODE_POLL );
    fmac_FilterState_t filterState = FMAC_FILTER_READY;

    UtFmac_ModelStart();
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_FMAC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_ResetCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() deinitializes the module when the unit does not finish its reset.
 *
 * \par Expected results
 * - The HW model is not active: FMAC_CR.RESET stays set, the wait times out.
 * - FMAC_REQUEST_ERROR, the unit is reset again through Rcc, the clock is disabled, the module is not initialized.
 */
void Ut_Fmac_Init_ResetNeverCompletes_DeinitializesAndReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t      config      = UtFmac_Get_Config( FMAC_XFER_MODE_POLL );
    fmac_FilterState_t filterState = FMAC_FILTER_READY;

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_FMAC, RCC_REQUEST_OK );
    UtFmac_Expect_Release( FMAC_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() deinitializes the module when the read-back of the registers is not as required.
 *
 * \par Expected results
 * - The HW model keeps the saturation flag set after the reset (a stuck flag): the read-back fails.
 * - FMAC_REQUEST_ERROR, the module is deinitialized (Rcc reset and clock off), the module is not initialized.
 */
void Ut_Fmac_Init_ReadBackMismatch_DeinitializesAndReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t      config      = UtFmac_Get_Config( FMAC_XFER_MODE_POLL );
    fmac_FilterState_t filterState = FMAC_FILTER_READY;

    UtFmac_ModelStart();
    utFmac_StuckSr = FMAC_SR_SAT;
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_FMAC, RCC_REQUEST_OK );
    UtFmac_Expect_Release( FMAC_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ResetCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() in interrupt mode deinitializes the module when Nvic rejects the handler.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR, the interrupt is disabled again in Nvic, the unit is reset through Rcc and the clock is
 *   disabled, the module is not initialized.
 */
void Ut_Fmac_Init_IsrNvicFails_DeinitializesAndReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t      config      = UtFmac_Get_Config( FMAC_XFER_MODE_ISR );
    fmac_FilterState_t filterState = FMAC_FILTER_READY;

    UtFmac_ModelStart();
    utFmac_NvicHandlerResult = NVIC_REQUEST_ERROR;
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_FMAC, RCC_REQUEST_OK );
    UtFmac_Expect_Release( FMAC_XFER_MODE_ISR );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Deinit() stops the filter, clears the control register and releases the peripheral.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, the function is stopped (START cleared), FMAC_CR is zero, Rcc reset active / inactive and the
 *   clock disabled, the callbacks are removed.
 * - Every other function returns FMAC_REQUEST_ERROR afterwards, a second Fmac_Deinit() too.
 */
void Ut_Fmac_Deinit_RunningFilter_ReleasesPeripheral( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    TEST_ASSERT_BITS_HIGH( FMAC_PARAM_START, FMAC->PARAM );

    UtFmac_Expect_Release( FMAC_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );

    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->CR );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStart() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Deinit() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Deinit() in interrupt mode disables the interrupt in Nvic.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, Nvic_Set_PeriphIrq_Inactive() is called for the FMAC interrupt.
 */
void Ut_Fmac_Deinit_Isr_DisablesInterrupt( void )
{
#if defined(FMAC)
    UtFmac_Init( FMAC_XFER_MODE_ISR );

    UtFmac_Expect_Release( FMAC_XFER_MODE_ISR );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Deinit() of a module which is not initialized returns an error.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR without any Rcc / Nvic call.
 */
void Ut_Fmac_Deinit_NotInitialized_ReturnsError( void )
{
#if defined(FMAC)
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Deinit() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* ============================= MEMORY USAGE =============================== */

/**
 * \brief   Fmac_Get_MemoryUsage() returns the words of the local memory a configuration needs.
 *
 * \par Expected results
 * - FIR with 4 coefficients: 4 (X2) + 5 (X1, headroom 1) + 1 (Y) words.
 * - IIR with 4 + 2 coefficients: 6 + 5 + 3 words.
 * - The watermark threshold is the headroom when it is above the requested one: watermark 8 on both buffers of the
 *   FIR gives 4 + 12 + 8 words; a requested headroom of 5 above the threshold is kept.
 * - The function needs no initialization and writes no register.
 */
void Ut_Fmac_Get_MemoryUsage_ValidConfigs_ReturnsWords( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t firConfig = UtFmac_Get_FirConfig();
    fmac_FilterConfig_t iirConfig = UtFmac_Get_IirConfig();
    fmac_SampleCnt_t    wordCnt   = 0u;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_MemoryUsage( &firConfig, &wordCnt ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_FIR_WORDS, wordCnt );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_MemoryUsage( &iirConfig, &wordCnt ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_IIR_WORDS, wordCnt );

    firConfig.InputWatermark  = FMAC_WATERMARK_8;
    firConfig.OutputWatermark = FMAC_WATERMARK_8;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_MemoryUsage( &firConfig, &wordCnt ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_FIR_WM8_WORDS, wordCnt );

    firConfig.InputHeadroom  = UT_FMAC_HEADROOM_SOME;
    firConfig.InputWatermark = FMAC_WATERMARK_4;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_MemoryUsage( &firConfig, &wordCnt ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_FIR_HEADROOM_WORDS, wordCnt );

    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->CR );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Get_MemoryUsage() returns the words also when the configuration does not fit the memory.
 *
 * \par Expected results
 * - FIR with 127 coefficients and a headroom of 2 words: FMAC_REQUEST_OK, 127 + 129 + 1 = 257 words (above 256).
 */
void Ut_Fmac_Get_MemoryUsage_TooLarge_ReturnsWordsAboveMemory( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t firConfig = UtFmac_Get_FirConfig();
    fmac_SampleCnt_t    wordCnt   = 0u;

    firConfig.BCount        = FMAC_FIR_COEF_MAX;
    firConfig.InputHeadroom = UT_FMAC_HEADROOM_OVER;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_MemoryUsage( &firConfig, &wordCnt ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_MEMORY_OVER, wordCnt );
    TEST_ASSERT_TRUE( FMAC_MEMORY_WORDS < wordCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Get_MemoryUsage() rejects an invalid configuration and NULL pointers.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR and the count is not written for: NULL configuration, NULL count, function out of the list,
 *   B NULL, 1 or 128 coefficients of a FIR, an IIR with 65 feedforward, 0 or 64 feedback coefficients or A NULL,
 *   gain / clipping / watermark out of the list, a headroom above the memory.
 */
void Ut_Fmac_Get_MemoryUsage_InvalidConfig_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t config  = UtFmac_Get_FirConfig();
    fmac_FilterConfig_t iirBase = UtFmac_Get_IirConfig();
    fmac_SampleCnt_t    wordCnt = UT_FMAC_MEMORY_OVER;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( NULL, &wordCnt ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, NULL ) );

    config.Function = FMAC_FUNC_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );

    config = UtFmac_Get_FirConfig();
    config.B = NULL;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );

    config = UtFmac_Get_FirConfig();
    config.BCount = FMAC_FIR_COEF_MIN - 1u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );
    config.BCount = UT_FMAC_TOO_MANY_COEF;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );

    config = iirBase;
    config.BCount = UT_FMAC_TOO_MANY_IIR_B;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );
    config = iirBase;
    config.ACount = 0u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );
    config.ACount = UT_FMAC_TOO_MANY_IIR_A;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );
    config = iirBase;
    config.A = NULL;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );

    config = UtFmac_Get_FirConfig();
    config.Gain = FMAC_GAIN_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );
    config = UtFmac_Get_FirConfig();
    config.Clipping = FMAC_CLIP_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );
    config = UtFmac_Get_FirConfig();
    config.InputWatermark = FMAC_WATERMARK_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );
    config = UtFmac_Get_FirConfig();
    config.OutputWatermark = FMAC_WATERMARK_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );
    config = UtFmac_Get_FirConfig();
    config.OutputHeadroom = UT_FMAC_HEADROOM_HUGE;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_MemoryUsage( &config, &wordCnt ) );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_MEMORY_OVER, wordCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* ============================ FILTER CONFIG =============================== */

/**
 * \brief   Fmac_Set_FilterConfig() of a FIR programs the buffers and loads the coefficients and the zero state.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, the unit is reset once more, FMAC_X2BUFCFG / X1BUFCFG / YBUFCFG hold the layout (X2 at 0 with
 *   4 words, X1 at 4 with 5 words, Y at 9 with 1 word), clipping is enabled.
 * - Two load functions ran: X2 with P = 4, Q = 0 and the coefficients, X1 with P = 3 and three zero samples
 *   (N - 1); no Y load, no function is running, no stray write; the filter state is FMAC_FILTER_READY.
 */
void Ut_Fmac_Set_FilterConfig_Fir_ProgramsBuffersAndLoadsCoefficients( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_UNCONFIGURED;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_RESETS_CONFIG, utFmac_ResetCnt );
    UtFmac_AssertBufCfg( FMAC->X2BUFCFG, UT_FMAC_FIR_X2_BASE, UT_FMAC_FIR_X2_SIZE, 0u );
    UtFmac_AssertBufCfg( FMAC->X1BUFCFG, UT_FMAC_FIR_X1_BASE, UT_FMAC_FIR_X1_SIZE, 0u );
    UtFmac_AssertBufCfg( FMAC->YBUFCFG, UT_FMAC_FIR_Y_BASE, UT_FMAC_FIR_Y_SIZE, 0u );
    TEST_ASSERT_EQUAL_HEX32( FMAC_CR_CLIPEN, FMAC->CR );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_LOADS_FIR, utFmac_StartCnt );
    TEST_ASSERT_EQUAL_UINT32( FMAC_REG_FUNC_LOAD_X2, utFmac_StartLog[ UT_FMAC_LOAD_X2_IDX ].Func );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS, utFmac_StartLog[ UT_FMAC_LOAD_X2_IDX ].ParamP );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StartLog[ UT_FMAC_LOAD_X2_IDX ].ParamQ );
    TEST_ASSERT_EQUAL_UINT32( FMAC_REG_FUNC_LOAD_X1, utFmac_StartLog[ UT_FMAC_LOAD_X1_IDX ].Func );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS - 1u, utFmac_StartLog[ UT_FMAC_LOAD_X1_IDX ].ParamP );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS, utFmac_X2Cnt );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_CoefB, utFmac_X2Data, UT_FMAC_TAPS );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS - 1u, utFmac_X1Cnt );
    for( uint32_t zeroIdx = 0u; ( UT_FMAC_TAPS - 1u ) > zeroIdx; zeroIdx ++ )
    {
        TEST_ASSERT_EQUAL_INT16( 0, utFmac_X1Data[ zeroIdx ] );
    }

    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_YCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StrayCnt );
    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_READY, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterConfig() of an IIR loads the feedback coefficients and the zero state of both buffers.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, layout X2 at 0 with 6 words, X1 at 6 with 5 words, Y at 11 with 3 words.
 * - Load X2 with P = 4, Q = 2: the feedforward coefficients followed by the feedback coefficients.
 * - Load X1 with three zero samples (N - 1) and load Y with P = 2 and two zero samples (M).
 */
void Ut_Fmac_Set_FilterConfig_Iir_LoadsFeedbackAndZeroState( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_IirConfig();

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );

    UtFmac_AssertBufCfg( FMAC->X2BUFCFG, 0u, UT_FMAC_IIR_X2_SIZE, 0u );
    UtFmac_AssertBufCfg( FMAC->X1BUFCFG, UT_FMAC_IIR_X1_BASE, UT_FMAC_IIR_X1_SIZE, 0u );
    UtFmac_AssertBufCfg( FMAC->YBUFCFG, UT_FMAC_IIR_Y_BASE, UT_FMAC_IIR_Y_SIZE, 0u );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_LOADS_IIR, utFmac_StartCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS, utFmac_StartLog[ UT_FMAC_LOAD_X2_IDX ].ParamP );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_FEEDBACK, utFmac_StartLog[ UT_FMAC_LOAD_X2_IDX ].ParamQ );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS + UT_FMAC_FEEDBACK, utFmac_X2Cnt );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_CoefB, utFmac_X2Data, UT_FMAC_TAPS );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_CoefA, &utFmac_X2Data[ UT_FMAC_TAPS ], UT_FMAC_FEEDBACK );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS - 1u, utFmac_X1Cnt );
    TEST_ASSERT_EQUAL_UINT32( FMAC_REG_FUNC_LOAD_Y, utFmac_StartLog[ UT_FMAC_LOAD_Y_IDX ].Func );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_FEEDBACK, utFmac_StartLog[ UT_FMAC_LOAD_Y_IDX ].ParamP );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_FEEDBACK, utFmac_YCnt );
    TEST_ASSERT_EQUAL_INT16( 0, utFmac_YData[ 0 ] );
    TEST_ASSERT_EQUAL_INT16( 0, utFmac_YData[ 1 ] );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StrayCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterConfig() programs the watermarks and the headroom and the clipping selection.
 *
 * \par Expected results
 * - Input watermark 4 and output watermark 8: FMAC_X1BUFCFG.FULL_WM = 2 and X1 size 4 + 4, FMAC_YBUFCFG.EMPTY_WM = 3
 *   and Y size 8, bases 4 and 12; the headroom of a requested 5 words above the watermark threshold of 4 wins.
 * - FMAC_CLIP_NONE: FMAC_CR.CLIPEN is cleared (it was set before).
 */
void Ut_Fmac_Set_FilterConfig_WatermarkHeadroomAndClipping_AreProgrammed( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    FMAC->CR = FMAC_CR_CLIPEN;

    filterConfig.InputWatermark  = FMAC_WATERMARK_4;
    filterConfig.OutputWatermark = FMAC_WATERMARK_8;
    filterConfig.Clipping        = FMAC_CLIP_NONE;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );

    UtFmac_AssertBufCfg( FMAC->X1BUFCFG, UT_FMAC_TAPS, UT_FMAC_WM_X1_SIZE, UT_FMAC_WM_4_CODE );
    UtFmac_AssertBufCfg( FMAC->YBUFCFG, UT_FMAC_WM_Y_BASE, UT_FMAC_WM_8_THRESHOLD, UT_FMAC_WM_8_CODE );
    TEST_ASSERT_BITS_LOW( FMAC_CR_CLIPEN, FMAC->CR );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );
    filterConfig.InputHeadroom = UT_FMAC_HEADROOM_SOME + UT_FMAC_WM_4_THRESHOLD;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );
    UtFmac_AssertBufCfg( FMAC->X1BUFCFG, UT_FMAC_TAPS, UT_FMAC_WM_HR_X1_SIZE, UT_FMAC_WM_4_CODE );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterConfig() accepts the largest filters which fit exactly into the 256 words.
 *
 * \par Expected results
 * - FIR with 127 coefficients: X2 127 words, X1 128 words, Y 1 word = 256 words: FMAC_REQUEST_OK, the Y buffer is
 *   at 255.
 * - IIR with 64 feedforward and 63 feedback coefficients: X2 127, X1 65, Y 64 = 256 words: FMAC_REQUEST_OK.
 */
void Ut_Fmac_Set_FilterConfig_LargestFilters_FitMemory( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t firConfig = UtFmac_Get_FirConfig();
    fmac_FilterConfig_t iirConfig = UtFmac_Get_IirConfig();

    UtFmac_Init( FMAC_XFER_MODE_POLL );

    firConfig.BCount = FMAC_FIR_COEF_MAX;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &firConfig ) );
    UtFmac_AssertBufCfg( FMAC->X1BUFCFG, UT_FMAC_FIR_MAX_X1_BASE, UT_FMAC_FIR_MAX_X1_SIZE, 0u );
    UtFmac_AssertBufCfg( FMAC->YBUFCFG, UT_FMAC_FIR_MAX_Y_BASE, UT_FMAC_FIR_Y_SIZE, 0u );
    TEST_ASSERT_EQUAL_UINT32( FMAC_FIR_COEF_MAX, utFmac_X2Cnt );

    iirConfig.BCount = UT_FMAC_IIR_MAX_B;
    iirConfig.ACount = UT_FMAC_IIR_MAX_A;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &iirConfig ) );
    UtFmac_AssertBufCfg( FMAC->X1BUFCFG, UT_FMAC_FIR_MAX_X1_BASE, UT_FMAC_IIR_MAX_X1_SIZE, 0u );
    UtFmac_AssertBufCfg( FMAC->YBUFCFG, UT_FMAC_IIR_MAX_Y_BASE, UT_FMAC_IIR_MAX_Y_SIZE, 0u );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StrayCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterConfig() rejects a filter which does not fit the local memory before a register is written.
 *
 * \par Expected results
 * - FIR with 127 coefficients and a headroom of 2 words (257 words): FMAC_REQUEST_ERROR.
 * - The unit is not reset again, no function is started, the buffer configuration registers stay zero, the filter
 *   stays FMAC_FILTER_UNCONFIGURED.
 */
void Ut_Fmac_Set_FilterConfig_OutOfMemory_ReturnsErrorWithoutRegisterWrite( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_READY;

    UtFmac_Init( FMAC_XFER_MODE_POLL );

    filterConfig.BCount        = FMAC_FIR_COEF_MAX;
    filterConfig.InputHeadroom = UT_FMAC_HEADROOM_OVER;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( &filterConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ResetCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StartCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->X1BUFCFG );
    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->X2BUFCFG );
    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->YBUFCFG );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_UNCONFIGURED, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterConfig() rejects an invalid configuration before a register is written.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR for NULL, B NULL, 1 coefficient, an IIR without feedback coefficients, an invalid gain;
 *   the unit is not reset again and no function is started.
 */
void Ut_Fmac_Set_FilterConfig_InvalidConfig_ReturnsErrorWithoutRegisterWrite( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t config = UtFmac_Get_FirConfig();

    UtFmac_Init( FMAC_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( NULL ) );

    config.B = NULL;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( &config ) );

    config = UtFmac_Get_FirConfig();
    config.BCount = FMAC_FIR_COEF_MIN - 1u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( &config ) );

    config = UtFmac_Get_IirConfig();
    config.ACount = 0u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( &config ) );

    config = UtFmac_Get_FirConfig();
    config.Gain = FMAC_GAIN_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ResetCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StartCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterConfig() needs an initialized module and a filter which does not run.
 *
 * \par Expected results
 * - Not initialized: FMAC_REQUEST_ERROR.
 * - A running filter is not reconfigured: FMAC_REQUEST_ERROR, the filter keeps running; after Fmac_Set_FilterStop()
 *   the configuration is accepted again.
 */
void Ut_Fmac_Set_FilterConfig_NotInitializedOrRunning_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_UNCONFIGURED;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( &filterConfig ) );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( &filterConfig ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_RUNNING, filterState );
    TEST_ASSERT_BITS_HIGH( FMAC_PARAM_START, FMAC->PARAM );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterConfig() fails and leaves the filter unconfigured when a load function never completes.
 *
 * \par Expected results
 * - The HW model never clears FMAC_PARAM.START after the load function: FMAC_REQUEST_ERROR and the state is
 *   FMAC_FILTER_UNCONFIGURED.
 */
void Ut_Fmac_Set_FilterConfig_LoadNeverCompletes_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_READY;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    utFmac_NoLoadDone = FMAC_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( &filterConfig ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_UNCONFIGURED, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   The module keeps a copy of the coefficients: the arrays of the application need not stay valid.
 *
 * \par Expected results
 * - After Fmac_Set_FilterConfig() the coefficient array is changed; Fmac_Set_FilterPreload() reloads the X2 buffer
 *   with the original coefficients.
 */
void Ut_Fmac_Set_FilterConfig_CopiesCoefficients_PreloadReloadsOriginal( void )
{
#if defined(FMAC)
    fmac_Sample_t       coefB[ UT_FMAC_TAPS ];
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();

    for( uint32_t coefIdx = 0u; UT_FMAC_TAPS > coefIdx; coefIdx ++ )
    {
        coefB[ coefIdx ] = utFmac_CoefB[ coefIdx ];
    }

    filterConfig.B = coefB;
    UtFmac_Init( FMAC_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );

    for( uint32_t coefIdx = 0u; UT_FMAC_TAPS > coefIdx; coefIdx ++ )
    {
        coefB[ coefIdx ] = 0;
    }

    utFmac_X2Cnt = 0u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS, utFmac_X2Cnt );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_CoefB, utFmac_X2Data, UT_FMAC_TAPS );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   The coefficients of a correlation and of a dot product are loaded in the reverse order.
 *
 * \par Expected results
 * - Correlation: X2 holds B[ 3 ], B[ 2 ], B[ 1 ], B[ 0 ] and the X1 buffer gets N - 1 zero samples is not loaded for
 *   a vector function: only the X2 load runs (a vector function loads its state with every request).
 * - Dot product: the same reversed coefficients, only the X2 load.
 * - Convolution: the coefficients in the given order.
 */
void Ut_Fmac_Set_FilterConfig_VectorFunctions_LoadCoefficientsOnly( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_Sample_t       reversed[ UT_FMAC_TAPS ];

    for( uint32_t coefIdx = 0u; UT_FMAC_TAPS > coefIdx; coefIdx ++ )
    {
        reversed[ coefIdx ] = utFmac_CoefB[ ( UT_FMAC_TAPS - 1u ) - coefIdx ];
    }

    UtFmac_Init( FMAC_XFER_MODE_POLL );

    filterConfig.Function = FMAC_FUNC_CORRELATION;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_StartCnt );
    TEST_ASSERT_EQUAL_INT16_ARRAY( reversed, utFmac_X2Data, UT_FMAC_TAPS );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_X1Cnt );

    utFmac_StartCnt = 0u;
    utFmac_X2Cnt    = 0u;
    filterConfig.Function = FMAC_FUNC_DOT_PRODUCT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_StartCnt );
    TEST_ASSERT_EQUAL_INT16_ARRAY( reversed, utFmac_X2Data, UT_FMAC_TAPS );

    utFmac_StartCnt = 0u;
    utFmac_X2Cnt    = 0u;
    filterConfig.Function = FMAC_FUNC_CONVOLUTION;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_StartCnt );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_CoefB, utFmac_X2Data, UT_FMAC_TAPS );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StrayCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* ============================ FILTER PRELOAD ============================== */

/**
 * \brief   Fmac_Set_FilterPreload() of a FIR loads the history of the input behind zero samples.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, the unit is reset, the coefficients are loaded again, X1 gets N - 1 = 3 samples: one zero
 *   sample and the two history samples, oldest first; no Y load.
 * - The filter is FMAC_FILTER_READY.
 */
void Ut_Fmac_Set_FilterPreload_Fir_LoadsHistoryBehindZeros( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_UNCONFIGURED;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Config( &filterConfig );
    utFmac_StartCnt = 0u;
    utFmac_X1Cnt    = 0u;
    utFmac_X2Cnt    = 0u;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterPreload( utFmac_HistoryIn, UT_FMAC_HISTORY_IN_CNT, NULL, 0u ) );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_RESETS_PRELOAD, utFmac_ResetCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_LOADS_FIR, utFmac_StartCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS, utFmac_X2Cnt );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS - 1u, utFmac_X1Cnt );
    TEST_ASSERT_EQUAL_INT16( 0, utFmac_X1Data[ 0 ] );
    TEST_ASSERT_EQUAL_INT16( utFmac_HistoryIn[ 0 ], utFmac_X1Data[ UT_FMAC_HISTORY_ZERO_CNT ] );
    TEST_ASSERT_EQUAL_INT16( utFmac_HistoryIn[ 1 ], utFmac_X1Data[ UT_FMAC_HISTORY_ZERO_CNT + 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_YCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_READY, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterPreload() of an IIR loads the history of the output behind zero samples.
 *
 * \par Expected results
 * - The Y buffer gets M = 2 samples: one zero sample and the history sample; X1 gets the history of the input behind
 *   zero samples as for the FIR.
 */
void Ut_Fmac_Set_FilterPreload_Iir_LoadsOutputHistory( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t    filterConfig = UtFmac_Get_IirConfig();
    const fmac_SampleCnt_t inCnt        = UT_FMAC_HISTORY_IN_CNT;
    const fmac_SampleCnt_t outCnt       = UT_FMAC_HISTORY_OUT_CNT;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Config( &filterConfig );
    utFmac_X1Cnt = 0u;
    utFmac_YCnt  = 0u;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterPreload( utFmac_HistoryIn, inCnt, utFmac_HistoryOut, outCnt ) );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS - 1u, utFmac_X1Cnt );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_FEEDBACK, utFmac_YCnt );
    TEST_ASSERT_EQUAL_INT16( 0, utFmac_YData[ 0 ] );
    TEST_ASSERT_EQUAL_INT16( utFmac_HistoryOut[ 0 ], utFmac_YData[ UT_FMAC_HISTORY_ZERO_CNT ] );
    TEST_ASSERT_EQUAL_INT16( utFmac_HistoryIn[ 1 ], utFmac_X1Data[ UT_FMAC_HISTORY_ZERO_CNT + 1u ] );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterPreload() rejects an invalid history before a register is written.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR without a reset of the unit for: a history of N samples (longer than N - 1), a count with a
 *   NULL pointer, an output history for a FIR, an output history longer than M for an IIR.
 */
void Ut_Fmac_Set_FilterPreload_InvalidHistory_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t    firConfig = UtFmac_Get_FirConfig();
    fmac_FilterConfig_t    iirConfig = UtFmac_Get_IirConfig();
    const fmac_SampleCnt_t outCnt    = UT_FMAC_HISTORY_OUT_CNT;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Config( &firConfig );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterPreload( utFmac_Input, UT_FMAC_TAPS, NULL, 0u ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterPreload( NULL, UT_FMAC_HISTORY_IN_CNT, NULL, 0u ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterPreload( NULL, 0u, utFmac_HistoryOut, outCnt ) );

    UtFmac_Config( &iirConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterPreload( NULL, 0u, utFmac_CoefB, UT_FMAC_FEEDBACK + 1u ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterPreload( NULL, 0u, NULL, UT_FMAC_HISTORY_OUT_CNT ) );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_RESETS_PRELOAD, utFmac_ResetCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterPreload() needs a configured filter which does not run.
 *
 * \par Expected results
 * - Not initialized, not configured, running: FMAC_REQUEST_ERROR.
 * - After Fmac_Set_FilterStop() (state FMAC_FILTER_STOPPED) it is accepted and the filter is FMAC_FILTER_READY.
 */
void Ut_Fmac_Set_FilterPreload_WrongState_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_UNCONFIGURED;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );

    UtFmac_Running( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_READY, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A vector function takes no history: Fmac_Set_FilterPreload() accepts only empty histories.
 *
 * \par Expected results
 * - Convolution: a history of 1 sample is rejected, the empty history is accepted and loads only the coefficients.
 */
void Ut_Fmac_Set_FilterPreload_VectorFunction_AcceptsOnlyEmptyHistory( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t    filterConfig = UtFmac_Get_FirConfig();
    const fmac_SampleCnt_t outCnt       = UT_FMAC_HISTORY_OUT_CNT;

    filterConfig.Function = FMAC_FUNC_CONVOLUTION;
    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Config( &filterConfig );
    utFmac_StartCnt = 0u;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterPreload( utFmac_HistoryIn, outCnt, NULL, 0u ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_StartCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* ============================ START AND STOP ============================== */

/**
 * \brief   Fmac_Set_FilterStart() starts the FIR function with the coefficient count and the gain.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, FMAC_PARAM = FUNC 8 (FIR), P = 4, Q = 0, R = gain 32 (5), START; the filter is running.
 * - In POLL mode FMAC_CR holds only the clipping bit, the transfer state is idle.
 */
void Ut_Fmac_Set_FilterStart_Fir_StartsFunction( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_UNCONFIGURED;

    filterConfig.Gain = FMAC_GAIN_32;
    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Config( &filterConfig );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );

    TEST_ASSERT_EQUAL_HEX32( UtFmac_Get_Param( FMAC_REG_FUNC_FIR, UT_FMAC_TAPS, 0u, FMAC_GAIN_32 ) | FMAC_PARAM_START,
                             FMAC->PARAM );
    TEST_ASSERT_EQUAL_HEX32( FMAC_CR_CLIPEN, FMAC->CR );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_RUNNING, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   The start of a filter function is accepted although the hardware stopped the function before the check.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, FMAC_PARAM holds FUNC 8 (FIR), P = 4, Q = 0, R = gain 1 and START is cleared, the filter is
 *   running (a vector function of a request which is read out by the interrupt or by the DMA at once is stopped by
 *   the end of its request before the start is checked).
 */
void Ut_Fmac_Set_FilterStart_StartClearedAtOnce_StillStarted( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_UNCONFIGURED;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Config( &filterConfig );
    utFmac_StartAutoDone = FMAC_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );

    TEST_ASSERT_EQUAL_HEX32( UtFmac_Get_Param( FMAC_REG_FUNC_FIR, UT_FMAC_TAPS, 0u, filterConfig.Gain ),
                             FMAC->PARAM );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_RUNNING, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterStart() starts the IIR function with both coefficient counts.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, FMAC_PARAM = FUNC 9 (IIR), P = 4, Q = 2, R = gain 1 (0), START.
 */
void Ut_Fmac_Set_FilterStart_Iir_StartsFunction( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_IirConfig();
    const uint32_t      funcParam = UtFmac_Get_Param( FMAC_REG_FUNC_IIR, UT_FMAC_TAPS, UT_FMAC_FEEDBACK, FMAC_GAIN_1 );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Config( &filterConfig );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );

    TEST_ASSERT_EQUAL_HEX32( funcParam | FMAC_PARAM_START, FMAC->PARAM );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   In interrupt mode Fmac_Set_FilterStart() enables the error interrupts of the unit.
 *
 * \par Expected results
 * - FMAC_CR holds CLIPEN, SATIEN, UNFLIEN and OVFLIEN (the data interrupts are enabled by a request).
 */
void Ut_Fmac_Set_FilterStart_Isr_EnablesErrorInterrupts( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();

    UtFmac_Init( FMAC_XFER_MODE_ISR );
    UtFmac_Config( &filterConfig );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );

    TEST_ASSERT_EQUAL_HEX32( FMAC_CR_CLIPEN | FMAC_CR_SATIEN | FMAC_CR_UNFLIEN | FMAC_CR_OVFLIEN, FMAC->CR );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A vector function does not start the hardware with Fmac_Set_FilterStart().
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, FMAC_PARAM.START stays cleared (every request starts the function), the filter is running.
 */
void Ut_Fmac_Set_FilterStart_VectorFunction_DoesNotStartHardware( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_UNCONFIGURED;

    filterConfig.Function = FMAC_FUNC_CORRELATION;
    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Config( &filterConfig );

    utFmac_StartCnt = 0u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );

    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StartCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_RUNNING, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterStart() needs a ready filter.
 *
 * \par Expected results
 * - Not initialized, not configured, already running and stopped filter: FMAC_REQUEST_ERROR, FMAC_PARAM unchanged.
 */
void Ut_Fmac_Set_FilterStart_WrongState_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStart() );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStart() );
    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->PARAM );

    UtFmac_Running( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStart() );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStart() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterStop() stops the function, disables the interrupts and requires a preload.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, START is cleared, the error and data interrupt enables of FMAC_CR are cleared (the clipping
 *   bit stays), the filter is FMAC_FILTER_STOPPED and can not be started until Fmac_Set_FilterPreload().
 * - After the preload and the start the filter runs again.
 */
void Ut_Fmac_Set_FilterStop_RunningFilter_StopsAndRequiresPreload( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_FilterState_t  filterState  = FMAC_FILTER_UNCONFIGURED;

    UtFmac_Init( FMAC_XFER_MODE_ISR );
    UtFmac_Running( &filterConfig );
    FMAC->CR |= FMAC_CR_WIEN | FMAC_CR_RIEN;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStop() );

    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
    TEST_ASSERT_EQUAL_HEX32( FMAC_CR_CLIPEN, FMAC->CR );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStart() );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_RUNNING, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterStop() of a filter which does not run returns an error.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR when not initialized, not configured and ready.
 */
void Ut_Fmac_Set_FilterStop_NotRunning_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStop() );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStop() );

    UtFmac_Ready( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStop() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterStop() ends a request in progress without a callback.
 *
 * \par Expected results
 * - A block request is in progress (state busy): after Fmac_Set_FilterStop() the transfer state is idle and no
 *   callback was called; Fmac_Task() does nothing more.
 */
void Ut_Fmac_Set_FilterStop_RequestInProgress_EndsRequestSilently( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    UtFmac_FillOutput();
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );

    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* ============================ BLOCK PROCESSING ============================ */

/**
 * \brief   A block request in POLL mode: Fmac_Task() moves the samples and the results, the callbacks are called.
 *
 * \par Expected results
 * - Fmac_Set_FilterProcess() returns FMAC_REQUEST_OK, the state is busy and nothing is moved before Fmac_Task().
 * - One Fmac_Task() (no flag set): the 6 samples are written to the running filter in the order of the input, the
 *   6 results (the value preset in FMAC_RDATA) are in the output buffer, no guard sample is overwritten.
 * - The half callback was called once (after 3 results), the complete callback once, the state is idle.
 */
void Ut_Fmac_Set_FilterProcess_Poll_MovesSamplesAndResults( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    UtFmac_FillOutput();
    FMAC->SR   = UT_FMAC_REG_ZERO;
    FMAC->RDATA = UT_FMAC_RESULT_RAW;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );

    Fmac_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT, utFmac_RunCnt );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_Input, utFmac_RunData, UT_FMAC_INPUT_CNT );
    UtFmac_AssertOutput( UT_FMAC_INPUT_CNT );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StrayCnt );

    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT, utFmac_RunCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   The flags decide how many samples Fmac_Task() moves: a full input buffer and an empty output buffer wait.
 *
 * \par Expected results
 * - X1FULL and YEMPTY set: Fmac_Task() moves nothing, the request stays busy.
 * - X1FULL cleared by the hardware after 2 samples (the HW model sets it again): only 2 samples are written.
 * - Then X1FULL is cleared again: all samples are written; with YEMPTY set no result is read yet.
 * - Finally YEMPTY is cleared: the results are read, the request completes (callbacks once).
 */
void Ut_Fmac_Set_FilterProcess_Poll_FlagsControlTheFlow( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    UtFmac_FillOutput();
    FMAC->RDATA = UT_FMAC_RESULT_RAW;
    FMAC->SR    = FMAC_SR_X1FULL | FMAC_SR_YEMPTY;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );

    utFmac_X1FullAfter = UT_FMAC_FULL_AFTER;
    FMAC->SR = FMAC_SR_YEMPTY;
    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_FULL_AFTER, utFmac_RunCnt );

    utFmac_X1FullAfter = 0u;
    FMAC->SR = FMAC_SR_YEMPTY;
    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT, utFmac_RunCnt );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_Input, utFmac_RunData, UT_FMAC_INPUT_CNT );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );

    FMAC->SR = UT_FMAC_REG_ZERO;
    Fmac_Task();
    UtFmac_AssertOutput( UT_FMAC_INPUT_CNT );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A block request in interrupt mode: the interrupts of the data are enabled and disabled by the handler.
 *
 * \par Expected results
 * - Fmac_Set_FilterProcess(): FMAC_CR gets WIEN and RIEN in addition to the error interrupts.
 * - The ISR with YEMPTY set: all samples written, WIEN is cleared (no more samples), RIEN stays, the request is busy.
 * - The ISR with YEMPTY cleared: the results are read, the complete callback is called, RIEN is cleared, idle.
 * - Fmac_Task() does not serve the request in interrupt mode.
 */
void Ut_Fmac_Set_FilterProcess_Isr_HandlerServesRequest( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;

    UtFmac_Init( FMAC_XFER_MODE_ISR );
    UtFmac_Running( &filterConfig );
    UtFmac_FillOutput();
    FMAC->RDATA = UT_FMAC_RESULT_RAW;
    FMAC->SR    = FMAC_SR_YEMPTY;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
    TEST_ASSERT_BITS_HIGH( FMAC_CR_WIEN | FMAC_CR_RIEN, FMAC->CR );

    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );

    TEST_ASSERT_NOT_NULL( utFmac_Isr );
    utFmac_Isr();
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT, utFmac_RunCnt );
    TEST_ASSERT_BITS_LOW( FMAC_CR_WIEN, FMAC->CR );
    TEST_ASSERT_BITS_HIGH( FMAC_CR_RIEN, FMAC->CR );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );

    FMAC->SR = UT_FMAC_REG_ZERO;
    utFmac_Isr();
    UtFmac_AssertOutput( UT_FMAC_INPUT_CNT );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_BITS_LOW( FMAC_CR_RIEN | FMAC_CR_WIEN, FMAC->CR );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterProcess() rejects invalid parameters and a wrong state of the module.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR for: not initialized, filter not running (ready), NULL input, NULL output, no input sample,
 *   an output buffer smaller than the count of the results; nothing is moved.
 * - A second request while one is in progress is rejected.
 */
void Ut_Fmac_Set_FilterProcess_InvalidRequest_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t    filterConfig = UtFmac_Get_FirConfig();
    fmac_Sample_t * const  output       = &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ];
    const fmac_SampleCnt_t sampleCnt    = UT_FMAC_INPUT_CNT;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartBlock() );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Ready( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartBlock() );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterProcess( NULL, sampleCnt, output, sampleCnt ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterProcess( utFmac_Input, sampleCnt, NULL, sampleCnt ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterProcess( utFmac_Input, 0u, output, sampleCnt ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterProcess( utFmac_Input, sampleCnt, output, sampleCnt - 1u ) );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartBlock() );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   An overflow of the input buffer ends the request and is reported by the error callback.
 *
 * \par Expected results
 * - FMAC_SR.OVFL set during the request: Fmac_Task() calls the error callback once with FMAC_XFER_ERROR_OVERFLOW,
 *   the state is FMAC_XFER_STATE_ERROR, Fmac_Get_XferError() returns the error, no complete callback.
 * - The function is stopped, the data interrupts are disabled, the filter is FMAC_FILTER_STOPPED.
 * - A new request is rejected; after Fmac_Set_FilterPreload() and Fmac_Set_FilterStart() it is accepted again.
 */
void Ut_Fmac_Set_FilterProcess_Overflow_ReportsErrorAndAbortsRequest( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_XferError_t    xferError    = FMAC_XFER_ERROR_NONE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_Init( FMAC_XFER_MODE_ISR );
    UtFmac_Running( &filterConfig );
    FMAC->SR = FMAC_SR_YEMPTY;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    FMAC->SR = FMAC_SR_YEMPTY | FMAC_SR_OVFL;
    TEST_ASSERT_NOT_NULL( utFmac_Isr );
    utFmac_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_OVERFLOW, utFmac_LastErrorMask );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_ERROR, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferError( &xferError ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_OVERFLOW, xferError );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
    TEST_ASSERT_BITS_LOW( FMAC_CR_WIEN | FMAC_CR_RIEN, FMAC->CR );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartBlock() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   An underflow of the output buffer ends the request and is reported by the error callback.
 *
 * \par Expected results
 * - FMAC_SR.UNFL set after the start of the request in POLL mode: Fmac_Task() calls the error callback once with
 *   FMAC_XFER_ERROR_UNDERFLOW, the state is FMAC_XFER_STATE_ERROR, the filter is stopped.
 * - Further calls of Fmac_Task() do not report it again.
 */
void Ut_Fmac_Set_FilterProcess_Underflow_ReportsErrorAndAbortsRequest( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    FMAC->SR = FMAC_SR_YEMPTY;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    FMAC->SR = FMAC_SR_YEMPTY | FMAC_SR_UNFL;
    Fmac_Task();
    Fmac_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_UNDERFLOW, utFmac_LastErrorMask );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_ERROR, xferState );
    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A saturation of the accumulator is reported once and the request goes on.
 *
 * \par Expected results
 * - FMAC_SR.SAT set during the request: the error callback is called once with FMAC_XFER_ERROR_SATURATION, the
 *   request is not aborted and completes normally (complete callback once), the error stays in Fmac_Get_XferError().
 */
void Ut_Fmac_Set_FilterProcess_Saturation_ReportsAndContinues( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;
    fmac_XferError_t    xferError    = FMAC_XFER_ERROR_NONE;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    FMAC->RDATA = UT_FMAC_RESULT_RAW;
    FMAC->SR    = FMAC_SR_YEMPTY;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    FMAC->SR = FMAC_SR_YEMPTY | FMAC_SR_SAT;
    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_SATURATION, utFmac_LastErrorMask );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );

    FMAC->SR = FMAC_SR_SAT;
    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferError( &xferError ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_SATURATION, xferError );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   An error flag which is already set when a request starts is not reported for that request.
 *
 * \par Expected results
 * - The flags are sticky until the unit is reset: FMAC_SR.SAT set before Fmac_Set_FilterProcess() is not reported,
 *   the request completes, Fmac_Get_XferError() is FMAC_XFER_ERROR_NONE.
 */
void Ut_Fmac_Set_FilterProcess_StickyFlagBeforeRequest_IsNotReported( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferError_t    xferError    = FMAC_XFER_ERROR_ALL;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    FMAC->RDATA = UT_FMAC_RESULT_RAW;
    FMAC->SR    = FMAC_SR_SAT;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
    Fmac_Task();

    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferError( &xferError ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_NONE, xferError );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Task() does nothing before the initialization and without a request.
 *
 * \par Expected results
 * - No register is changed, no callback is called.
 */
void Ut_Fmac_Task_NotInitializedOrIdle_DoesNothing( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();

    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_ResetCnt );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    FMAC->SR = UT_FMAC_REG_ZERO;
    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StrayCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* ======================== STATE GETTERS AND INTERRUPT ===================== */

/**
 * \brief   The state getters reject a NULL pointer and a module which is not initialized.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR for Fmac_Get_FilterState(), Fmac_Get_XferState() and Fmac_Get_XferError() before the
 *   initialization (the value is not written) and for a NULL pointer after it.
 */
void Ut_Fmac_Get_State_NullOrNotInitialized_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterState_t filterState = FMAC_FILTER_READY;
    fmac_XferState_t   xferState   = FMAC_XFER_STATE_BUSY;
    fmac_XferError_t   xferError   = FMAC_XFER_ERROR_ALL;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_XferError( &xferError ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_READY, filterState );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_ALL, xferError );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( NULL ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_XferState( NULL ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_XferError( NULL ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_IrqPriority() and Fmac_Get_IrqPriority() use the Nvic priority of the FMAC interrupt.
 *
 * \par Expected results
 * - After the initialization the priority is passed to Nvic and read from it, an Nvic error is returned as an error.
 * - Before the initialization both functions return FMAC_REQUEST_ERROR without an Nvic call, Get with NULL too.
 */
void Ut_Fmac_IrqPriority_SetAndGet_UseNvic( void )
{
#if defined(FMAC)
    fmac_IrqPrio_t irqPrio = 0u;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_IrqPriority( UT_FMAC_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_IrqPriority( &irqPrio ) );

    UtFmac_Init( FMAC_XFER_MODE_ISR );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_FMAC, UT_FMAC_IRQ_PRIO, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_IrqPriority( UT_FMAC_IRQ_PRIO ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_FMAC, UT_FMAC_IRQ_PRIO, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_IrqPriority( UT_FMAC_IRQ_PRIO ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_FMAC, &irqPrio, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_IrqPriority( &irqPrio ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_FMAC, &irqPrio, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_IrqPriority( &irqPrio ) );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_IrqPriority( NULL ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* ============================== CONVERSION ================================ */

/**
 * \brief   Fmac_Get_FromFloat() converts to q1.15 with rounding to the nearest and saturation.
 *
 * \par Expected results
 * - 0.5 is 16384, -0.5 is -16384, 0 is 0, 1.0 and above saturate to 32767, -1.0 and below to -32768, the largest
 *   representable float below 1 is 32767.
 * - 0.4 LSB rounds to 0, 0.6 LSB to 1, -0.6 LSB to -1.
 * - A NaN and a NULL pointer are rejected, the sample is not written.
 * - No initialization is needed.
 */
void Ut_Fmac_Get_FromFloat_ConvertsRoundsAndSaturates( void )
{
#if defined(FMAC)
    fmac_Sample_t sample     = 0;
    const float   notANumber = ( UT_FMAC_FLOAT_ZERO / UT_FMAC_FLOAT_ZERO );
    const float   lsbLow     = ( UT_FMAC_FLOAT_LSB * UT_FMAC_FLOAT_LSB_FRACTION_LOW );
    const float   lsbHigh    = ( UT_FMAC_FLOAT_LSB * UT_FMAC_FLOAT_LSB_FRACTION_HIGH );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( UT_FMAC_FLOAT_HALF, &sample ) );
    TEST_ASSERT_EQUAL_INT16( UT_FMAC_SAMPLE_HALF, sample );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( UT_FMAC_FLOAT_HALF_NEG, &sample ) );
    TEST_ASSERT_EQUAL_INT16( -UT_FMAC_SAMPLE_HALF, sample );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( UT_FMAC_FLOAT_ZERO, &sample ) );
    TEST_ASSERT_EQUAL_INT16( 0, sample );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( UT_FMAC_FLOAT_ONE, &sample ) );
    TEST_ASSERT_EQUAL_INT16( UT_FMAC_SAMPLE_MAX, sample );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( UT_FMAC_FLOAT_TWO, &sample ) );
    TEST_ASSERT_EQUAL_INT16( UT_FMAC_SAMPLE_MAX, sample );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( UT_FMAC_FLOAT_ONE_NEG, &sample ) );
    TEST_ASSERT_EQUAL_INT16( UT_FMAC_SAMPLE_MIN, sample );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( UT_FMAC_FLOAT_TWO_NEG, &sample ) );
    TEST_ASSERT_EQUAL_INT16( UT_FMAC_SAMPLE_MIN, sample );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( UT_FMAC_FLOAT_LARGEST, &sample ) );
    TEST_ASSERT_EQUAL_INT16( UT_FMAC_SAMPLE_MAX, sample );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( lsbLow, &sample ) );
    TEST_ASSERT_EQUAL_INT16( 0, sample );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( lsbHigh, &sample ) );
    TEST_ASSERT_EQUAL_INT16( UT_FMAC_SAMPLE_ONE_LSB, sample );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( -lsbHigh, &sample ) );
    TEST_ASSERT_EQUAL_INT16( -UT_FMAC_SAMPLE_ONE_LSB, sample );

    sample = UT_FMAC_SAMPLE_ONE_LSB;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FromFloat( notANumber, &sample ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FromFloat( UT_FMAC_FLOAT_HALF, NULL ) );
    TEST_ASSERT_EQUAL_INT16( UT_FMAC_SAMPLE_ONE_LSB, sample );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Get_ToFloat() converts a q1.15 sample to a floating point value.
 *
 * \par Expected results
 * - 16384 is 0.5, -32768 is -1.0, 0 is 0.0, 1 is 2^-15; a NULL pointer is rejected.
 */
void Ut_Fmac_Get_ToFloat_ConvertsSamples( void )
{
#if defined(FMAC)
    float value = UT_FMAC_FLOAT_ONE;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_ToFloat( UT_FMAC_SAMPLE_HALF, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_FMAC_FLOAT_HALF, value );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_ToFloat( UT_FMAC_SAMPLE_MIN, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_FMAC_FLOAT_ONE_NEG, value );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_ToFloat( 0, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_FMAC_FLOAT_ZERO, value );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_ToFloat( UT_FMAC_SAMPLE_ONE_LSB, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_FMAC_FLOAT_LSB, value );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_ToFloat( UT_FMAC_SAMPLE_HALF, NULL ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* ============================== STREAMS =================================== */

/**
 * \brief   A circular stream in POLL mode runs one lap of the buffers per Fmac_Task() until it is stopped.
 *
 * \par Expected results
 * - Fmac_Set_StreamStart() returns FMAC_REQUEST_OK, the state is busy and nothing is moved before Fmac_Task().
 * - Every Fmac_Task() writes the 6 input samples in the order of the buffer (from its begin again in the second lap)
 *   and reads 6 results; the half callback and the complete callback are called once per lap, the state stays busy.
 * - Fmac_Set_StreamStop() returns FMAC_REQUEST_OK, the state is idle, the filter is FMAC_FILTER_STOPPED, the function
 *   is stopped and Fmac_Task() moves nothing more; a second Fmac_Set_StreamStop() returns FMAC_REQUEST_ERROR.
 */
void Ut_Fmac_Set_StreamStart_CircularPoll_RunsLapsUntilStopped( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    UtFmac_FillOutput();
    FMAC->SR    = UT_FMAC_REG_ZERO;
    FMAC->RDATA = UT_FMAC_RESULT_RAW;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );

    for( uint32_t lapIdx = 0u; UT_FMAC_STREAM_LAPS > lapIdx; lapIdx ++ )
    {
        Fmac_Task();

        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT * ( lapIdx + 1u ), utFmac_RunCnt );
        TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_Input,
                                       &utFmac_RunData[ UT_FMAC_INPUT_CNT * lapIdx ],
                                       UT_FMAC_INPUT_CNT );
        UtFmac_AssertOutput( UT_FMAC_INPUT_CNT );
        TEST_ASSERT_EQUAL_UINT32( lapIdx + 1u, utFmac_HalfCnt );
        TEST_ASSERT_EQUAL_UINT32( lapIdx + 1u, utFmac_CompleteCnt );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
        TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );
    }

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_StreamStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );

    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT * UT_FMAC_STREAM_LAPS, utFmac_RunCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_STREAM_LAPS, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStop() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A one shot stream in POLL mode ends after one pass through the buffers.
 *
 * \par Expected results
 * - One Fmac_Task() writes the 6 input samples, reads the 6 results, calls the half and the complete callback once.
 * - The state is idle and the filter keeps running (a one shot stream is not stopped), the next Fmac_Task() moves
 *   nothing, Fmac_Set_StreamStop() returns FMAC_REQUEST_ERROR (no stream in progress).
 * - A new stream is accepted.
 */
void Ut_Fmac_Set_StreamStart_OneShotPoll_EndsAfterOnePass( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;
    fmac_FilterState_t  filterState  = FMAC_FILTER_STOPPED;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    UtFmac_FillOutput();
    FMAC->SR    = UT_FMAC_REG_ZERO;
    FMAC->RDATA = UT_FMAC_RESULT_RAW;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_ONE_SHOT ) );
    Fmac_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT, utFmac_RunCnt );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_Input, utFmac_RunData, UT_FMAC_INPUT_CNT );
    UtFmac_AssertOutput( UT_FMAC_INPUT_CNT );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_RUNNING, filterState );

    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT, utFmac_RunCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_ONE_SHOT ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A one shot stream of an IIR filter writes the samples and reads the results like a FIR filter.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, one Fmac_Task() moves the 6 samples and the 6 results, the complete callback is called once,
 *   the state is idle.
 */
void Ut_Fmac_Set_StreamStart_IirOneShot_ProcessesSamples( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_IirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    UtFmac_FillOutput();
    FMAC->SR    = UT_FMAC_REG_ZERO;
    FMAC->RDATA = UT_FMAC_RESULT_RAW;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_ONE_SHOT ) );
    Fmac_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT, utFmac_RunCnt );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_Input, utFmac_RunData, UT_FMAC_INPUT_CNT );
    UtFmac_AssertOutput( UT_FMAC_INPUT_CNT );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A circular stream in interrupt mode is served by the handler, one lap per interrupt.
 *
 * \par Expected results
 * - Fmac_Set_StreamStart(): FMAC_CR gets WIEN and RIEN, nothing is moved before the interrupt.
 * - Every call of the ISR writes the 6 samples (WIEN stays set, the stream goes on) and reads the 6 results, the
 *   complete callback is called once per lap.
 * - Fmac_Set_StreamStop() clears the data and the error interrupt enables (the clipping bit stays).
 */
void Ut_Fmac_Set_StreamStart_CircularIsr_HandlerServesLaps( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();

    UtFmac_Init( FMAC_XFER_MODE_ISR );
    UtFmac_Running( &filterConfig );
    UtFmac_FillOutput();
    FMAC->SR    = UT_FMAC_REG_ZERO;
    FMAC->RDATA = UT_FMAC_RESULT_RAW;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );
    TEST_ASSERT_BITS_HIGH( FMAC_CR_WIEN | FMAC_CR_RIEN, FMAC->CR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );

    TEST_ASSERT_NOT_NULL( utFmac_Isr );

    for( uint32_t lapIdx = 0u; UT_FMAC_STREAM_LAPS > lapIdx; lapIdx ++ )
    {
        utFmac_Isr();

        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT * ( lapIdx + 1u ), utFmac_RunCnt );
        TEST_ASSERT_EQUAL_UINT32( lapIdx + 1u, utFmac_CompleteCnt );
        TEST_ASSERT_BITS_HIGH( FMAC_CR_WIEN | FMAC_CR_RIEN, FMAC->CR );
    }

    UtFmac_AssertOutput( UT_FMAC_INPUT_CNT );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_StreamStop() );
    TEST_ASSERT_EQUAL_HEX32( FMAC_CR_CLIPEN, FMAC->CR );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A callback can stop a circular stream: the processing of the lap ends.
 *
 * \par Expected results
 * - The complete callback calls Fmac_Set_StreamStop(): after the first Fmac_Task() the state is idle, the filter is
 *   FMAC_FILTER_STOPPED, the complete callback was called once and no second lap is processed.
 */
void Ut_Fmac_Set_StreamStart_CallbackStopsStream_EndsProcessing( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    FMAC->SR    = UT_FMAC_REG_ZERO;
    FMAC->RDATA = UT_FMAC_RESULT_RAW;
    utFmac_StopInComplete = FMAC_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );
    Fmac_Task();
    Fmac_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT, utFmac_RunCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_StreamStart() rejects a wrong state of the module and invalid requests.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR when not initialized and when the filter does not run (ready).
 * - FMAC_REQUEST_ERROR for: NULL request, NULL input, NULL output, a size of 1 sample, different sizes, a buffer
 *   mode out of the list; the unit is not reset by a rejected request.
 * - A valid request is accepted, a second one while the stream is in progress is rejected.
 */
void Ut_Fmac_Set_StreamStart_InvalidRequest_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t  filterConfig  = UtFmac_Get_FirConfig();
    fmac_StreamRequest_t streamRequest = UtFmac_Get_StreamRequest( FMAC_BUFFER_CIRCULAR );
    uint32_t             resetCnt      = 0u;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Ready( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );
    resetCnt = utFmac_ResetCnt;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStart( NULL ) );

    streamRequest.Input = NULL;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStart( &streamRequest ) );

    streamRequest = UtFmac_Get_StreamRequest( FMAC_BUFFER_CIRCULAR );
    streamRequest.Output = NULL;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStart( &streamRequest ) );

    streamRequest = UtFmac_Get_StreamRequest( FMAC_BUFFER_CIRCULAR );
    streamRequest.InputSize  = 1u;
    streamRequest.OutputSize = 1u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStart( &streamRequest ) );

    streamRequest = UtFmac_Get_StreamRequest( FMAC_BUFFER_CIRCULAR );
    streamRequest.OutputSize = UT_FMAC_INPUT_CNT - 1u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStart( &streamRequest ) );

    streamRequest = UtFmac_Get_StreamRequest( FMAC_BUFFER_CIRCULAR );
    streamRequest.BufferMode = FMAC_BUFFER_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStart( &streamRequest ) );
    TEST_ASSERT_EQUAL_UINT32( resetCnt, utFmac_ResetCnt );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   The vector functions do not stream.
 *
 * \par Expected results
 * - Dot product, convolution and correlation: Fmac_Set_StreamStart() returns FMAC_REQUEST_ERROR, the unit is not
 *   reset, the transfer state stays idle.
 */
void Ut_Fmac_Set_StreamStart_VectorFunction_ReturnsError( void )
{
#if defined(FMAC)
    const fmac_Function_t vectorFunctions[] = { FMAC_FUNC_DOT_PRODUCT, FMAC_FUNC_CONVOLUTION, FMAC_FUNC_CORRELATION };
    fmac_XferState_t      xferState         = FMAC_XFER_STATE_BUSY;

    UtFmac_Init( FMAC_XFER_MODE_POLL );

    for( uint32_t funcIdx = 0u; ( sizeof( vectorFunctions ) / sizeof( vectorFunctions[ 0 ] ) ) > funcIdx; funcIdx ++ )
    {
        fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
        uint32_t            resetCnt     = 0u;

        filterConfig.Function = vectorFunctions[ funcIdx ];
        UtFmac_Ready( &filterConfig );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );
        resetCnt = utFmac_ResetCnt;

        TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartStream( FMAC_BUFFER_ONE_SHOT ) );
        TEST_ASSERT_EQUAL_UINT32( resetCnt, utFmac_ResetCnt );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
        TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStop() );
    }
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_StreamStop() needs a stream in progress.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR when not initialized, when the filter is idle and when a block request (not a stream) is in
 *   progress; the block request is not disturbed and completes.
 */
void Ut_Fmac_Set_StreamStop_NoStream_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStop() );

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    FMAC->SR    = UT_FMAC_REG_ZERO;
    FMAC->RDATA = UT_FMAC_RESULT_RAW;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStop() );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStop() );

    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   An overflow ends a stream: the error is reported, the stream is not stopped by the application.
 *
 * \par Expected results
 * - FMAC_SR.OVFL set during a circular stream in interrupt mode: the error callback is called once with
 *   FMAC_XFER_ERROR_OVERFLOW, the transfer state is FMAC_XFER_STATE_ERROR, the filter is FMAC_FILTER_STOPPED and the
 *   data interrupts are disabled.
 * - Fmac_Set_StreamStop() returns FMAC_REQUEST_ERROR afterwards (no stream in progress).
 */
void Ut_Fmac_Set_StreamStart_Overflow_AbortsStream( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_Init( FMAC_XFER_MODE_ISR );
    UtFmac_Running( &filterConfig );
    FMAC->SR    = UT_FMAC_REG_ZERO;
    FMAC->RDATA = UT_FMAC_RESULT_RAW;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );

    FMAC->SR = FMAC_SR_OVFL;
    TEST_ASSERT_NOT_NULL( utFmac_Isr );
    utFmac_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_OVERFLOW, utFmac_LastErrorMask );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_ERROR, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
    TEST_ASSERT_BITS_LOW( FMAC_CR_WIEN | FMAC_CR_RIEN, FMAC->CR );
    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStop() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* ========================== VECTOR FUNCTIONS ============================== */

/**
 * \brief   A dot product request loads its inputs into the unit and reads one result.
 *
 * \par Expected results
 * - Fmac_Set_FilterProcess() with 4 inputs (the count of the coefficients) and an output buffer of 1 sample: the unit
 *   is reset, X2 is loaded with the coefficients (in the reversed order), X1 with the 4 inputs, then the FIR
 *   function of the hardware starts with P = 4 and Q = 0; no sample is written by the data transfer.
 * - Fmac_Task() reads the one result (the preset value) into the output buffer, the complete callback is called once
 *   (no half callback: the result is one sample), the function is stopped, the state is idle.
 * - A second request does the same again (the unit is reset again).
 */
void Ut_Fmac_Set_FilterProcess_DotProduct_LoadsInputsAndReadsOneResult( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_VectorConfig( FMAC_FUNC_DOT_PRODUCT );
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    UtFmac_FillOutput();
    FMAC->RDATA = UT_FMAC_RESULT_RAW;

    for( uint32_t requestIdx = 0u; UT_FMAC_STREAM_LAPS > requestIdx; requestIdx ++ )
    {
        utFmac_ResetCnt = 0u;
        utFmac_StartCnt = 0u;
        utFmac_X1Cnt    = 0u;
        utFmac_X2Cnt    = 0u;
        utFmac_RunCnt   = 0u;

        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartDotProduct() );
        TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ResetCnt );
        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_VECTOR_STARTS, utFmac_StartCnt );
        TEST_ASSERT_EQUAL_UINT32( FMAC_REG_FUNC_LOAD_X2, utFmac_StartLog[ UT_FMAC_LOAD_X2_IDX ].Func );
        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS, utFmac_StartLog[ UT_FMAC_LOAD_X2_IDX ].ParamP );
        TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_CoefBRev, utFmac_X2Data, UT_FMAC_TAPS );
        TEST_ASSERT_EQUAL_UINT32( FMAC_REG_FUNC_LOAD_X1, utFmac_StartLog[ UT_FMAC_LOAD_X1_IDX ].Func );
        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS, utFmac_StartLog[ UT_FMAC_LOAD_X1_IDX ].ParamP );
        TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_Input, utFmac_X1Data, UT_FMAC_TAPS );
        TEST_ASSERT_EQUAL_UINT32( FMAC_REG_FUNC_FIR, utFmac_StartLog[ UT_FMAC_RUN_IDX ].Func );
        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS, utFmac_StartLog[ UT_FMAC_RUN_IDX ].ParamP );
        TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StartLog[ UT_FMAC_RUN_IDX ].ParamQ );
        TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );

        FMAC->SR = UT_FMAC_REG_ZERO;
        Fmac_Task();

        TEST_ASSERT_EQUAL_INT16( UT_FMAC_RESULT, utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ] );
        TEST_ASSERT_EQUAL_INT16( UT_FMAC_FILL_PATTERN, utFmac_OutputMem[ UT_FMAC_GUARD_SIZE + 1u ] );
        TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );
        TEST_ASSERT_EQUAL_UINT32( requestIdx + 1u, utFmac_CompleteCnt );
        TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_HalfCnt );
        TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
        TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    }
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A convolution request starts from the zero state: N - 1 zero samples, then the samples are streamed.
 *
 * \par Expected results
 * - Fmac_Set_FilterProcess() with 6 samples: the unit is reset, X2 holds the coefficients in their order, X1 gets 3
 *   zero samples, the FIR function of the hardware starts with P = 4.
 * - Fmac_Task() writes the 6 samples to the running function and reads 6 results; the half and the complete callback
 *   are called once, the function is stopped and the state is idle.
 * - A second request loads the zero state again (the unit is reset again, X1 gets 3 zero samples again).
 */
void Ut_Fmac_Set_FilterProcess_Convolution_StartsFromZeroState( void )
{
#if defined(FMAC)
    UtFmac_AssertVectorRequests( FMAC_FUNC_CONVOLUTION );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A correlation request starts from the zero state like a convolution, with reversed coefficients.
 *
 * \par Expected results
 * - Same as the convolution, X2 holds the coefficients in the reversed order (the hardware has no correlation
 *   function: it is the FIR function with the reversed vector).
 */
void Ut_Fmac_Set_FilterProcess_Correlation_StartsFromZeroState( void )
{
#if defined(FMAC)
    UtFmac_AssertVectorRequests( FMAC_FUNC_CORRELATION );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterProcess() of a dot product needs exactly as many inputs as coefficients.
 *
 * \par Expected results
 * - 3 or 5 inputs of a 4 coefficient dot product: FMAC_REQUEST_ERROR, the unit is not reset.
 */
void Ut_Fmac_Set_FilterProcess_DotProductWrongInputCount_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_VectorConfig( FMAC_FUNC_DOT_PRODUCT );
    fmac_Sample_t * const output     = &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ];

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    utFmac_ResetCnt = 0u;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterProcess( utFmac_Input, UT_FMAC_TAPS - 1u, output, 1u ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterProcess( utFmac_Input, UT_FMAC_TAPS + 1u, output, 1u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_ResetCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A vector request fails when the unit does not complete a load function.
 *
 * \par Expected results
 * - The HW model never completes the load functions: Fmac_Set_FilterProcess() returns FMAC_REQUEST_ERROR, the
 *   transfer state is FMAC_XFER_STATE_ERROR, the filter is FMAC_FILTER_STOPPED.
 * - After Fmac_Set_FilterPreload() and Fmac_Set_FilterStart() (the model completes the functions again) the state is
 *   idle and a request is accepted.
 */
void Ut_Fmac_Set_FilterProcess_VectorLoadNeverCompletes_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_VectorConfig( FMAC_FUNC_CONVOLUTION );
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );

    utFmac_NoLoadDone = FMAC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartBlock() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_ERROR, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );

    utFmac_NoLoadDone = FMAC_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* ================================ DMA MODE ================================ */

/**
 * \brief   Fmac_Init() in DMA mode initializes the two Dma streams and enables the interrupt in Nvic.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK; Dma calls in this order: init of the stream of the input data (stream 0), its transfer error
 *   interrupt and its interrupt line, init of the stream of the results (stream 1), its transfer error, transfer
 *   complete and half transfer interrupt and its interrupt line.
 * - Both streams: DMA1, low priority, normal mode, 16-bit data, fixed peripheral and incrementing memory address, the
 *   placeholder transfer of one sample, the transfer error callback; the input data go from the memory to FMAC_WDATA
 *   with the DMAMUX1 request FMAC_WRITE and have no other callback, the results go from FMAC_RDATA to the memory with
 *   FMAC_READ and have the complete and the half callback.
 * - Nvic: handler, priority and enable of the FMAC interrupt; FMAC_CR is zero.
 */
void Ut_Fmac_Init_Dma_InitializesDmaStreams( void )
{
#if defined(FMAC)
    uint32_t logIdx = 0u;

    UtFmac_Init( FMAC_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INIT_DMA_CALLS, utFmac_DmaCnt );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_INIT, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_TE_IRQ, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_IRQ_ACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_INIT, DMA_PERIPH_1, DMA_STREAM_1, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_TE_IRQ, DMA_PERIPH_1, DMA_STREAM_1, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_TC_IRQ, DMA_PERIPH_1, DMA_STREAM_1, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_HT_IRQ, DMA_PERIPH_1, DMA_STREAM_1, 0u );
    UtFmac_AssertDma( logIdx, UT_FMAC_DMA_IRQ_ACTIVE, DMA_PERIPH_1, DMA_STREAM_1, 0u );

    for( uint32_t streamIdx = 0u; UT_FMAC_DMA_STREAMS > streamIdx; streamIdx ++ )
    {
        TEST_ASSERT_EQUAL( DMA_PERIPH_1, utFmac_DmaInitConfig[ streamIdx ].DmaPeriphId );
        TEST_ASSERT_EQUAL( DMA_PRIORITY_LOW, utFmac_DmaInitConfig[ streamIdx ].Priority );
    }

    UtFmac_AssertInitStream( 0u,
                             DMA_STREAM_0,
                             DMA_DIR_MEMORY_TO_PERIPH,
                             DMA_REQ_FMAC_WRITE,
                             UT_FMAC_ADDR( &FMAC->WDATA ) );
    UtFmac_AssertInitStream( 1u,
                             DMA_STREAM_1,
                             DMA_DIR_PERIPH_TO_MEMORY,
                             DMA_REQ_FMAC_READ,
                             UT_FMAC_ADDR( &FMAC->RDATA ) );
    TEST_ASSERT_NULL( utFmac_DmaInitConfig[ 0u ].TransferCompleteCallback );
    TEST_ASSERT_NULL( utFmac_DmaInitConfig[ 0u ].HalfTransferCallback );
    TEST_ASSERT_NOT_NULL( utFmac_DmaInitConfig[ 1u ].TransferCompleteCallback );
    TEST_ASSERT_NOT_NULL( utFmac_DmaInitConfig[ 1u ].HalfTransferCallback );

    TEST_ASSERT_NOT_NULL( utFmac_Isr );
    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->CR );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() in DMA mode uses the streams of the configuration and enables a half transfer interrupt only
 *          for a configured half callback.
 *
 * \par Expected results
 * - Streams 3 (input data) and 4 (results) of DMA2 with high priority are initialized.
 * - Without a half callback in the configuration the stream of the results has no half transfer callback and its
 *   half transfer interrupt is not enabled.
 */
void Ut_Fmac_Init_Dma_ConfiguredChannels_AreUsed( void )
{
#if defined(FMAC)
    fmac_Config_t config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    uint32_t      logIdx = 0u;

    config.DmaPeriphId       = DMA_PERIPH_2;
    config.DmaWriteChannelId = DMA_STREAM_3;
    config.DmaReadChannelId  = DMA_STREAM_4;
    config.DmaPriority       = DMA_PRIORITY_HIGH;
    config.XferHalfCallback  = NULL;

    UtFmac_ModelStart();
    UtFmac_Expect_Init( FMAC_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Init( &config ) );

    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_INIT, DMA_PERIPH_2, DMA_STREAM_3, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_TE_IRQ, DMA_PERIPH_2, DMA_STREAM_3, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_IRQ_ACTIVE, DMA_PERIPH_2, DMA_STREAM_3, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_INIT, DMA_PERIPH_2, DMA_STREAM_4, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_TE_IRQ, DMA_PERIPH_2, DMA_STREAM_4, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_TC_IRQ, DMA_PERIPH_2, DMA_STREAM_4, 0u );
    UtFmac_AssertDma( logIdx, UT_FMAC_DMA_IRQ_ACTIVE, DMA_PERIPH_2, DMA_STREAM_4, 0u );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_DmaIdCnt[ UT_FMAC_DMA_HT_IRQ ] );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_HIGH, utFmac_DmaInitConfig[ 0u ].Priority );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_HIGH, utFmac_DmaInitConfig[ 1u ].Priority );
    TEST_ASSERT_NOT_NULL( utFmac_DmaInitConfig[ 1u ].TransferCompleteCallback );
    TEST_ASSERT_NULL( utFmac_DmaInitConfig[ 1u ].HalfTransferCallback );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() rejects an invalid DMA configuration.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR without any Rcc / Nvic / Dma call for: the same stream for both directions, a stream out of
 *   the list (input data and results), a Dma controller out of the list, a priority out of the list.
 */
void Ut_Fmac_Init_Dma_InvalidConfig_ReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );

    config.DmaReadChannelId = config.DmaWriteChannelId;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    config.DmaWriteChannelId = DMA_STREAM_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    config.DmaReadChannelId = DMA_STREAM_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    config.DmaPeriphId = DMA_PERIPH_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    config.DmaPriority = DMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_DmaCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_ResetCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() in DMA mode deinitializes the module when Dma rejects the second stream.
 *
 * \par Expected results
 * - Dma_Init() of the second stream fails: FMAC_REQUEST_ERROR, the Nvic interrupt is disabled, the stream which was
 *   initialized is stopped and released (only that one: stopped again and its interrupt line disabled), the unit is
 *   reset through Rcc and the clock is disabled, the module is not initialized.
 */
void Ut_Fmac_Init_Dma_DmaInitFails_DeinitializesAndReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t      config      = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    fmac_FilterState_t filterState = FMAC_FILTER_READY;
    uint32_t           logIdx      = 0u;

    UtFmac_ModelStart();
    utFmac_DmaFailId  = UT_FMAC_DMA_INIT;
    utFmac_DmaFailNth = UT_FMAC_TWICE;
    UtFmac_Expect_Init( FMAC_XFER_MODE_DMA );
    UtFmac_Expect_Release( FMAC_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INIT_FAIL_DMA_CALLS, utFmac_DmaCnt );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_INIT, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_TE_IRQ, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_IRQ_ACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_INIT, DMA_PERIPH_1, DMA_STREAM_1, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_XFER_INACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_XFER_INACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx, UT_FMAC_DMA_IRQ_INACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() in DMA mode deinitializes the module when Dma can not enable the interrupt line of a stream.
 *
 * \par Expected results
 * - Dma_Set_InterruptActive() of the first stream fails: FMAC_REQUEST_ERROR, the stream is stopped and released, the
 *   second stream is not initialized, the module is not initialized.
 */
void Ut_Fmac_Init_Dma_DmaInterruptFails_DeinitializesAndReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t      config      = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    fmac_FilterState_t filterState = FMAC_FILTER_READY;
    uint32_t           logIdx      = UT_FMAC_INIT_WRITE_CALLS;

    UtFmac_ModelStart();
    utFmac_DmaFailId  = UT_FMAC_DMA_IRQ_ACTIVE;
    utFmac_DmaFailNth = 1u;
    UtFmac_Expect_Init( FMAC_XFER_MODE_DMA );
    UtFmac_Expect_Release( FMAC_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_DmaIdCnt[ UT_FMAC_DMA_INIT ] );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_XFER_INACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_XFER_INACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx, UT_FMAC_DMA_IRQ_INACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Deinit() in DMA mode stops and releases both Dma streams.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK; Dma calls in this order: both streams stopped, then the stream of the input data and the stream
 *   of the results released (stopped and the interrupt line disabled).
 * - The interrupt is disabled in Nvic, the unit is reset and the clock disabled through Rcc.
 */
void Ut_Fmac_Deinit_Dma_ReleasesDmaStreams( void )
{
#if defined(FMAC)
    uint32_t logIdx = 0u;

    UtFmac_InitDma();

    UtFmac_Expect_Release( FMAC_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_DEINIT_DMA_CALLS, utFmac_DmaCnt );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_XFER_INACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_XFER_INACTIVE, DMA_PERIPH_1, DMA_STREAM_1, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_XFER_INACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_IRQ_INACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_XFER_INACTIVE, DMA_PERIPH_1, DMA_STREAM_1, 0u );
    UtFmac_AssertDma( logIdx, UT_FMAC_DMA_IRQ_INACTIVE, DMA_PERIPH_1, DMA_STREAM_1, 0u );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Deinit() in DMA mode returns an error when Dma can not release a stream, the module is released.
 *
 * \par Expected results
 * - Dma_Set_InterruptInactive() of the first stream fails: FMAC_REQUEST_ERROR, the second stream is released anyway,
 *   the unit is released through Rcc, every function returns FMAC_REQUEST_ERROR afterwards.
 */
void Ut_Fmac_Deinit_Dma_DmaReleaseFails_ReturnsErrorAndReleases( void )
{
#if defined(FMAC)
    fmac_FilterState_t filterState = FMAC_FILTER_READY;

    UtFmac_InitDma();
    utFmac_DmaFailId  = UT_FMAC_DMA_IRQ_INACTIVE;
    utFmac_DmaFailNth = 1u;

    UtFmac_Expect_Release( FMAC_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Deinit() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_DmaIdCnt[ UT_FMAC_DMA_IRQ_INACTIVE ] );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterStart() in DMA mode enables the DMA requests and the error interrupts.
 *
 * \par Expected results
 * - FMAC_CR has DMAWEN, DMAREN and the three error interrupt enables, WIEN and RIEN stay clear.
 */
void Ut_Fmac_Set_FilterStart_Dma_EnablesDmaRequests( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );

    TEST_ASSERT_BITS_HIGH( FMAC_CR_DMAWEN | FMAC_CR_DMAREN | UT_FMAC_CR_ERROR_IRQS, FMAC->CR );
    TEST_ASSERT_BITS_LOW( FMAC_CR_WIEN | FMAC_CR_RIEN, FMAC->CR );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A block request in DMA mode arms the two Dma streams, the CPU moves no sample.
 *
 * \par Expected results
 * - Fmac_Set_FilterProcess() returns FMAC_REQUEST_OK; for the stream of the results and then for the stream of the
 *   input data: the normal mode, the memory address (the output buffer, the input buffer), the count of 6 samples, the
 *   enable of the stream - in this order.
 * - FMAC_CR keeps WIEN and RIEN clear, Fmac_Task() moves nothing, the state is busy.
 */
void Ut_Fmac_Set_FilterProcess_Dma_ArmsDmaStreams( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    UtFmac_ResetDmaLog();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_DMA_STREAMS * UT_FMAC_ARM_CALLS, utFmac_DmaCnt );
    UtFmac_AssertArm( 0u,
                      DMA_STREAM_1,
                      DMA_TRANSFER_MODE_NORMAL,
                      UT_FMAC_ADDR( &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ] ),
                      UT_FMAC_INPUT_CNT );
    UtFmac_AssertArm( UT_FMAC_ARM_CALLS,
                      DMA_STREAM_0,
                      DMA_TRANSFER_MODE_NORMAL,
                      UT_FMAC_ADDR( utFmac_Input ),
                      UT_FMAC_INPUT_CNT );

    TEST_ASSERT_BITS_LOW( FMAC_CR_WIEN | FMAC_CR_RIEN, FMAC->CR );
    Fmac_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   The Dma callbacks of the results end a block request in DMA mode.
 *
 * \par Expected results
 * - The half transfer callback calls the half callback once, the request goes on.
 * - The transfer complete callback calls the complete callback once, the state is idle, the filter keeps running.
 * - A further transfer complete callback without a request does nothing.
 */
void Ut_Fmac_Set_FilterProcess_Dma_DmaCallbacksEndRequest( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_STOPPED;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    utFmac_DmaInitConfig[ 1u ].HalfTransferCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );

    utFmac_DmaInitConfig[ 1u ].TransferCompleteCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_RUNNING, filterState );

    utFmac_DmaInitConfig[ 1u ].TransferCompleteCallback();
    utFmac_DmaInitConfig[ 1u ].HalfTransferCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_HalfCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A request of the largest count a Dma stream moves is accepted, a larger one is rejected in DMA mode.
 *
 * \par Expected results
 * - FMAC_DMA_SAMPLE_CNT_MAX + 1 samples: FMAC_REQUEST_ERROR, no Dma call, the stream request too.
 * - FMAC_DMA_SAMPLE_CNT_MAX samples: FMAC_REQUEST_OK, the count of the stream of the results is the count of the
 *   samples.
 */
void Ut_Fmac_Set_FilterProcess_Dma_SizeLimit_IsChecked( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t  filterConfig  = UtFmac_Get_FirConfig();
    fmac_StreamRequest_t streamRequest = UtFmac_Get_StreamRequest( FMAC_BUFFER_ONE_SHOT );
    fmac_Sample_t * const output       = &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ];

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    UtFmac_ResetDmaLog();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR,
                       Fmac_Set_FilterProcess( utFmac_Input,
                                               FMAC_DMA_SAMPLE_CNT_MAX + 1u,
                                               output,
                                               FMAC_DMA_SAMPLE_CNT_MAX + 1u ) );
    streamRequest.InputSize  = FMAC_DMA_SAMPLE_CNT_MAX + 1u;
    streamRequest.OutputSize = FMAC_DMA_SAMPLE_CNT_MAX + 1u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStart( &streamRequest ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_DmaCnt );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK,
                       Fmac_Set_FilterProcess( utFmac_Input,
                                               FMAC_DMA_SAMPLE_CNT_MAX,
                                               output,
                                               FMAC_DMA_SAMPLE_CNT_MAX ) );
    UtFmac_AssertDma( UT_FMAC_COUNT_IDX,
                      UT_FMAC_DMA_COUNT,
                      DMA_PERIPH_1,
                      DMA_STREAM_1,
                      FMAC_DMA_SAMPLE_CNT_MAX );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A circular stream in DMA mode uses the circular mode of the Dma streams and reports every lap.
 *
 * \par Expected results
 * - Fmac_Set_StreamStart(): both streams are armed in the circular mode with 6 samples.
 * - Every transfer complete callback of the results calls the complete callback, the state stays busy; the half
 *   transfer callback calls the half callback.
 * - Fmac_Set_StreamStop() stops both Dma streams, clears the DMA request enables of FMAC_CR, the state is idle, the
 *   filter is FMAC_FILTER_STOPPED.
 */
void Ut_Fmac_Set_StreamStart_CircularDma_UsesCircularMode( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    UtFmac_ResetDmaLog();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );
    UtFmac_AssertArm( 0u,
                      DMA_STREAM_1,
                      DMA_TRANSFER_MODE_CIRCULAR,
                      UT_FMAC_ADDR( &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ] ),
                      UT_FMAC_INPUT_CNT );
    UtFmac_AssertArm( UT_FMAC_ARM_CALLS,
                      DMA_STREAM_0,
                      DMA_TRANSFER_MODE_CIRCULAR,
                      UT_FMAC_ADDR( utFmac_Input ),
                      UT_FMAC_INPUT_CNT );

    for( uint32_t lapIdx = 0u; UT_FMAC_STREAM_LAPS > lapIdx; lapIdx ++ )
    {
        utFmac_DmaInitConfig[ 1u ].HalfTransferCallback();
        utFmac_DmaInitConfig[ 1u ].TransferCompleteCallback();

        TEST_ASSERT_EQUAL_UINT32( lapIdx + 1u, utFmac_HalfCnt );
        TEST_ASSERT_EQUAL_UINT32( lapIdx + 1u, utFmac_CompleteCnt );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
        TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );
    }

    UtFmac_ResetDmaLog();
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_StreamStop() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_DmaCnt );
    UtFmac_AssertDma( 0u, UT_FMAC_DMA_XFER_INACTIVE, DMA_PERIPH_1, DMA_STREAM_0, 0u );
    UtFmac_AssertDma( 1u, UT_FMAC_DMA_XFER_INACTIVE, DMA_PERIPH_1, DMA_STREAM_1, 0u );
    TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );
    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_StreamStop() in DMA mode returns an error when Dma can not stop a stream, the stream is ended.
 *
 * \par Expected results
 * - Dma_Set_TransferInactive() of the first stream fails: FMAC_REQUEST_ERROR, the second stream is stopped anyway,
 *   the state is idle and the filter is FMAC_FILTER_STOPPED.
 */
void Ut_Fmac_Set_StreamStop_Dma_DmaStopFails_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );

    UtFmac_ResetDmaLog();
    utFmac_DmaFailId  = UT_FMAC_DMA_XFER_INACTIVE;
    utFmac_DmaFailNth = 1u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStop() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_DmaIdCnt[ UT_FMAC_DMA_XFER_INACTIVE ] );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterStop() in DMA mode stops the Dma streams of a request in progress.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, both streams are stopped, no callback is called, the state is idle, the DMA request enables of
 *   FMAC_CR are cleared and the filter is FMAC_FILTER_STOPPED.
 */
void Ut_Fmac_Set_FilterStop_Dma_RequestInProgress_StopsStreams( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    UtFmac_ResetDmaLog();
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStop() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_DmaIdCnt[ UT_FMAC_DMA_XFER_INACTIVE ] );
    TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A Dma transfer error ends a request in DMA mode with FMAC_XFER_ERROR_DMA.
 *
 * \par Expected results
 * - The transfer error callback of a stream is called during a block request: the error callback of the module is
 *   called once with FMAC_XFER_ERROR_DMA, the Dma streams are stopped, the transfer state is FMAC_XFER_STATE_ERROR,
 *   the filter is FMAC_FILTER_STOPPED, the DMA request enables are cleared.
 * - A transfer complete callback afterwards does nothing; a transfer error callback without a request is ignored.
 */
void Ut_Fmac_Set_FilterProcess_Dma_DmaError_ReportsAndAbortsRequest( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_XferError_t    xferError    = FMAC_XFER_ERROR_NONE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    UtFmac_ResetDmaLog();
    utFmac_DmaInitConfig[ 0u ].TransferErrorCallback();

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_DMA, utFmac_LastErrorMask );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_DmaIdCnt[ UT_FMAC_DMA_XFER_INACTIVE ] );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_ERROR, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferError( &xferError ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_DMA, xferError );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
    TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );

    utFmac_DmaInitConfig[ 1u ].TransferCompleteCallback();
    utFmac_DmaInitConfig[ 1u ].TransferErrorCallback();
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   The FMAC interrupt in DMA mode reports the error flags, the data are moved by the Dma streams.
 *
 * \par Expected results
 * - No request in progress: the interrupt with the overflow flag set does nothing (no callback).
 * - A request in progress and FMAC_SR.SAT set: the error callback is called once with FMAC_XFER_ERROR_SATURATION, the
 *   request goes on (busy), the saturation interrupt is disabled.
 * - FMAC_SR.OVFL set: the error callback is called with FMAC_XFER_ERROR_OVERFLOW, the request ends with an error and
 *   the Dma streams are stopped.
 * - The interrupt never moves a sample.
 */
void Ut_Fmac_Isr_Dma_ReportsErrorFlags( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    TEST_ASSERT_NOT_NULL( utFmac_Isr );

    FMAC->SR = FMAC_SR_OVFL;
    utFmac_Isr();
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_ErrorCnt );

    FMAC->SR = UT_FMAC_REG_ZERO;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    FMAC->SR = FMAC_SR_SAT;
    utFmac_Isr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_SATURATION, utFmac_LastErrorMask );
    TEST_ASSERT_BITS_LOW( FMAC_CR_SATIEN, FMAC->CR );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );

    UtFmac_ResetDmaLog();
    FMAC->SR = FMAC_SR_SAT | FMAC_SR_OVFL;
    utFmac_Isr();
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_OVERFLOW, utFmac_LastErrorMask );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_DmaIdCnt[ UT_FMAC_DMA_XFER_INACTIVE ] );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_ERROR, xferState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A step of the arming of the Dma streams which fails ends the request with an error.
 *
 * \par Expected results
 * - For each of the Dma calls of the arming (transfer mode, memory address, count, enable of the stream of the results
 *   and of the input data) failing in turn: Fmac_Set_FilterProcess() returns FMAC_REQUEST_ERROR, the transfer state
 *   is FMAC_XFER_STATE_ERROR, the filter is FMAC_FILTER_STOPPED and the DMA request enables are cleared.
 * - After the preload and the start a request is accepted again.
 */
void Ut_Fmac_Set_FilterProcess_Dma_ArmingFails_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );

    for( uint32_t stepIdx = 0u; UT_FMAC_ARM_FAIL_CNT > stepIdx; stepIdx ++ )
    {
        UtFmac_ResetDmaLog();
        utFmac_DmaFailId  = utFmac_ArmFailLut[ stepIdx ].Id;
        utFmac_DmaFailNth = utFmac_ArmFailLut[ stepIdx ].Nth;

        TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartBlock() );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
        TEST_ASSERT_EQUAL( FMAC_XFER_STATE_ERROR, xferState );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
        TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
        TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );

        utFmac_DmaFailId  = UT_FMAC_DMA_NONE;
        utFmac_DmaFailNth = 0u;
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );
    }

    UtFmac_ResetDmaLog();
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A dot product in DMA mode arms only the stream of the results.
 *
 * \par Expected results
 * - Fmac_Set_FilterProcess() with 4 inputs: the inputs are loaded into X1 by the CPU, the stream of the results is
 *   armed (one result), the stream of the input data is not, FMAC_CR has DMAREN and no DMAWEN.
 * - The transfer complete callback ends the request: the complete callback is called once, the state is idle, the
 *   function is stopped and the DMA request enables are cleared.
 */
void Ut_Fmac_Set_FilterProcess_DotProductDma_ArmsResultChannelOnly( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_VectorConfig( FMAC_FUNC_DOT_PRODUCT );
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    UtFmac_ResetDmaLog();
    utFmac_X1Cnt = 0u;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartDotProduct() );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_Input, utFmac_X1Data, UT_FMAC_TAPS );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_ARM_CALLS, utFmac_DmaCnt );
    UtFmac_AssertArm( 0u,
                      DMA_STREAM_1,
                      DMA_TRANSFER_MODE_NORMAL,
                      UT_FMAC_ADDR( &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ] ),
                      UT_FMAC_DOT_RESULT_CNT );
    TEST_ASSERT_BITS_HIGH( FMAC_CR_DMAREN, FMAC->CR );
    TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN, FMAC->CR );

    utFmac_DmaInitConfig[ 1u ].TransferCompleteCallback();

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
    TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A convolution in DMA mode arms both streams and enables both DMA requests.
 *
 * \par Expected results
 * - Fmac_Set_FilterProcess() with 6 samples: both streams are armed, FMAC_CR has DMAWEN and DMAREN, the CPU writes no
 *   sample to the running function.
 * - The transfer complete callback ends the request, the function is stopped and the DMA request enables are cleared.
 */
void Ut_Fmac_Set_FilterProcess_ConvolutionDma_ArmsBothChannels( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_VectorConfig( FMAC_FUNC_CONVOLUTION );
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    UtFmac_ResetDmaLog();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_DMA_STREAMS * UT_FMAC_ARM_CALLS, utFmac_DmaCnt );
    TEST_ASSERT_BITS_HIGH( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );

    utFmac_DmaInitConfig[ 1u ].TransferCompleteCallback();

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
    TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}

/* =========================== LOCAL FUNCTIONS ============================== */

#if defined(FMAC)

/**
 * \brief HW model: emulates the reset of the unit, the completion of a load function, the recording of the data
 *        written to FMAC_WDATA and X1FULL (see the file header). Called after every store to the registers.
 */
static void UtFmac_HwModel( void )
{
    const uint32_t crValue = FMAC->CR;

    if( 0u != ( crValue & FMAC_CR_RESET ) )
    {
        utFmac_ResetCnt ++;
        utFmac_LoadRemaining = 0u;
        utFmac_LastParam     = 0u;

        ( void )__atomic_and_fetch( &FMAC->CR, ~FMAC_CR_RESET, __ATOMIC_SEQ_CST );
        __atomic_store_n( &FMAC->PARAM, 0u, __ATOMIC_SEQ_CST );
        __atomic_store_n( &FMAC->SR, ( FMAC_SR_YEMPTY | utFmac_StuckSr ), __ATOMIC_SEQ_CST );
    }
    else
    {
        /* No reset requested */
    }

    const uint32_t paramValue = FMAC->PARAM;

    if( paramValue != utFmac_LastParam )
    {
        utFmac_LastParam = paramValue;

        if( 0u != ( paramValue & FMAC_PARAM_START ) )
        {
            const uint32_t func = ( ( paramValue & FMAC_PARAM_FUNC_Msk ) >> FMAC_PARAM_FUNC_Pos );
            const uint32_t parP = ( ( paramValue & FMAC_PARAM_P_Msk ) >> FMAC_PARAM_P_Pos );
            const uint32_t parQ = ( ( paramValue & FMAC_PARAM_Q_Msk ) >> FMAC_PARAM_Q_Pos );
            const uint32_t parR = ( ( paramValue & FMAC_PARAM_R_Msk ) >> FMAC_PARAM_R_Pos );

            if( UT_FMAC_START_LOG_SIZE > utFmac_StartCnt )
            {
                utFmac_StartLog[ utFmac_StartCnt ].Func   = func;
                utFmac_StartLog[ utFmac_StartCnt ].ParamP = parP;
                utFmac_StartLog[ utFmac_StartCnt ].ParamQ = parQ;
                utFmac_StartLog[ utFmac_StartCnt ].ParamR = parR;
            }
            else
            {
                /* The log is full */
            }

            utFmac_StartCnt ++;
            utFmac_LoadFunc = func;

            if( FMAC_REG_FUNC_LOAD_X1 == func )
            {
                utFmac_LoadRemaining = parP;
            }
            else if( FMAC_REG_FUNC_LOAD_Y == func )
            {
                utFmac_LoadRemaining = parP;
            }
            else if( FMAC_REG_FUNC_LOAD_X2 == func )
            {
                utFmac_LoadRemaining = ( parP + parQ );
            }
            else
            {
                utFmac_LoadRemaining = 0u;

                if( FMAC_FUNCTION_ACTIVE == utFmac_StartAutoDone )
                {
                    /* The request of the function ended and stopped it before the software checked the start */
                    utFmac_LastParam = ( paramValue & ~FMAC_PARAM_START );
                    __atomic_store_n( &FMAC->PARAM, utFmac_LastParam, __ATOMIC_SEQ_CST );
                }
                else
                {
                    /* The function runs until it is stopped */
                }
            }
        }
        else
        {
            /* The function was stopped */
            utFmac_LoadRemaining = 0u;
        }
    }
    else
    {
        /* No new function */
    }

    const uint32_t wdataValue = FMAC->WDATA;

    if( UT_FMAC_WDATA_EMPTY != wdataValue )
    {
        __atomic_store_n( &FMAC->WDATA, UT_FMAC_WDATA_EMPTY, __ATOMIC_SEQ_CST );
        UtFmac_ModelLogSample( (fmac_Sample_t)(uint16_t)wdataValue );
    }
    else
    {
        /* No sample was written */
    }
}


/**
 * \brief Appends a sample to a log of the HW model (a full log keeps its content, the count goes on).
 *
 * \param logData [in]: Log of the samples.
 * \param logCnt  [in]: Pointer to the count of the samples written to the log.
 * \param sample  [in]: Sample to append.
 */
static void UtFmac_ModelAppend( fmac_Sample_t * const logData,
                                uint32_t * const logCnt,
                                fmac_Sample_t sample )
{
    if( UT_FMAC_LOG_SIZE > *logCnt )
    {
        logData[ *logCnt ] = sample;
    }
    else
    {
        /* The log is full */
    }

    *logCnt = ( *logCnt + 1u );
}


/**
 * \brief Records a sample written to FMAC_WDATA by the load function in progress or by the running filter, completes
 *        a load function after its last sample and sets X1FULL after the configured count of samples.
 *
 * \param sample [in]: Sample written to FMAC_WDATA.
 */
static void UtFmac_ModelLogSample( fmac_Sample_t sample )
{
    const uint32_t       paramValue = FMAC->PARAM;
    const uint32_t       runFunc    = ( ( paramValue & FMAC_PARAM_FUNC_Msk ) >> FMAC_PARAM_FUNC_Pos );
    fmac_FunctionState_t runActive  = FMAC_FUNCTION_INACTIVE;

    if( 0u != ( paramValue & FMAC_PARAM_START ) )
    {
        if( FMAC_REG_FUNC_FIR == runFunc )
        {
            runActive = FMAC_FUNCTION_ACTIVE;
        }
        else if( FMAC_REG_FUNC_IIR == runFunc )
        {
            runActive = FMAC_FUNCTION_ACTIVE;
        }
        else
        {
            /* A load function is not a filter */
        }
    }
    else
    {
        /* No function runs */
    }

    if( 0u != utFmac_LoadRemaining )
    {
        if( FMAC_REG_FUNC_LOAD_X1 == utFmac_LoadFunc )
        {
            UtFmac_ModelAppend( utFmac_X1Data, &utFmac_X1Cnt, sample );
        }
        else if( FMAC_REG_FUNC_LOAD_X2 == utFmac_LoadFunc )
        {
            UtFmac_ModelAppend( utFmac_X2Data, &utFmac_X2Cnt, sample );
        }
        else
        {
            UtFmac_ModelAppend( utFmac_YData, &utFmac_YCnt, sample );
        }

        utFmac_LoadRemaining --;

        if( 0u == utFmac_LoadRemaining )
        {
            if( FMAC_FUNCTION_INACTIVE == utFmac_NoLoadDone )
            {
                ( void )__atomic_and_fetch( &FMAC->PARAM, ~FMAC_PARAM_START, __ATOMIC_SEQ_CST );
                utFmac_LastParam = FMAC->PARAM;
            }
            else
            {
                /* The model never completes the load function */
            }
        }
        else
        {
            /* The load function goes on */
        }
    }
    else if( FMAC_FUNCTION_ACTIVE == runActive )
    {
        UtFmac_ModelAppend( utFmac_RunData, &utFmac_RunCnt, sample );

        if( 0u != utFmac_X1FullAfter )
        {
            if( utFmac_RunCnt >= utFmac_X1FullAfter )
            {
                ( void )__atomic_or_fetch( &FMAC->SR, FMAC_SR_X1FULL, __ATOMIC_SEQ_CST );
            }
            else
            {
                /* Fewer samples than the full buffer holds */
            }
        }
        else
        {
            /* X1FULL is not emulated */
        }
    }
    else
    {
        utFmac_StrayCnt ++;
    }
}


/**
 * \brief Prepares the HW model: empties FMAC_WDATA (the model detects the writes by a change of the value) and
 *        activates the model.
 */
static void UtFmac_ModelStart( void )
{
    FMAC->WDATA = UT_FMAC_WDATA_EMPTY;
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtFmac_HwModel ) );
}


/** Stub of Nvic_Set_PeriphIrq_Handler: stores the registered FMAC ISR and returns the configured result */
static nvic_RequestState_t UtFmac_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId,
                                                      const nvic_IsrCallback_t irqHandler,
                                                      int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_FMAC, irqId );

    utFmac_Isr = irqHandler;

    return ( utFmac_NvicHandlerResult );
}


/** Half of the output buffer callback of the tests: counts the calls */
static void UtFmac_XferHalfCallback( void )
{
    utFmac_HalfCnt ++;
}


/** End of the output buffer callback of the tests: counts the calls, stops the stream when the test asks for it */
static void UtFmac_XferCompleteCallback( void )
{
    utFmac_CompleteCnt ++;

    if( FMAC_FUNCTION_ACTIVE == utFmac_StopInComplete )
    {
        ( void )Fmac_Set_StreamStop();
    }
    else
    {
        /* The callback does not stop the stream */
    }
}


/** Error callback of the tests: counts the calls and keeps the last error mask */
static void UtFmac_ErrorCallback( fmac_XferError_t errorMask )
{
    utFmac_ErrorCnt ++;
    utFmac_LastErrorMask = errorMask;
}


/**
 * \brief Returns the default configuration with the given transfer mode, the IRQ priority and the test callbacks.
 *
 * \param xferMode [in]: Transfer mode.
 *
 * \return Configuration for Fmac_Init().
 */
static fmac_Config_t UtFmac_Get_Config( fmac_XferMode_t xferMode )
{
    fmac_Config_t config;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_DefaultConfig( &config ) );

    config.XferMode             = xferMode;
    config.IrqPriority          = UT_FMAC_IRQ_PRIO;
    config.XferHalfCallback     = UtFmac_XferHalfCallback;
    config.XferCompleteCallback = UtFmac_XferCompleteCallback;
    config.ErrorCallback        = UtFmac_ErrorCallback;

    return ( config );
}


/**
 * \brief Expects the Rcc / Nvic calls of Fmac_Init() after the clock was enabled: nothing in POLL mode, the
 *        priority and the enable of the interrupt in interrupt and DMA mode (the handler is taken by the stub).
 *
 * \param xferMode [in]: Transfer mode.
 */
static void UtFmac_Expect_Init( fmac_XferMode_t xferMode )
{
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_FMAC, RCC_REQUEST_OK );

    if( FMAC_XFER_MODE_POLL != xferMode )
    {
        Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_FMAC, UT_FMAC_IRQ_PRIO, NVIC_REQUEST_OK );
        Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_FMAC, NVIC_REQUEST_OK );
    }
    else
    {
        /* POLL mode does not use the interrupt */
    }
}


/**
 * \brief Expects the Rcc / Nvic calls of the release of the peripheral (Fmac_Deinit() and a failed Fmac_Init()).
 *
 * \param xferMode [in]: Transfer mode the module was initialized with.
 */
static void UtFmac_Expect_Release( fmac_XferMode_t xferMode )
{
    if( FMAC_XFER_MODE_POLL != xferMode )
    {
        Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_FMAC, NVIC_REQUEST_OK );
    }
    else
    {
        /* POLL mode does not use the interrupt */
    }

    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_FMAC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( RCC_PERIPH_FMAC, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_FMAC, RCC_REQUEST_OK );
}


/**
 * \brief Activates the HW model and initializes the module in the given transfer mode.
 *
 * \param xferMode [in]: Transfer mode.
 */
static void UtFmac_Init( fmac_XferMode_t xferMode )
{
    fmac_Config_t config = UtFmac_Get_Config( xferMode );

    UtFmac_ModelStart();
    UtFmac_Expect_Init( xferMode );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Init( &config ) );
}


/** Returns the configuration of the FIR test filter: 4 coefficients, gain 1, clipping, watermark 1 */
static fmac_FilterConfig_t UtFmac_Get_FirConfig( void )
{
    fmac_FilterConfig_t filterConfig = { .Function = FMAC_FUNC_FIR, .B = utFmac_CoefB, .BCount = UT_FMAC_TAPS };

    return ( filterConfig );
}


/** Returns the configuration of the IIR test filter: 4 feedforward and 2 feedback coefficients */
static fmac_FilterConfig_t UtFmac_Get_IirConfig( void )
{
    fmac_FilterConfig_t filterConfig =
    {
        .Function = FMAC_FUNC_IIR,
        .B        = utFmac_CoefB,
        .BCount   = UT_FMAC_TAPS,
        .A        = utFmac_CoefA,
        .ACount   = UT_FMAC_FEEDBACK,
    };

    return ( filterConfig );
}


/**
 * \brief Configures the filter (the module is initialized) and checks the result.
 *
 * \param filterConfig [in]: Filter configuration.
 */
static void UtFmac_Config( const fmac_FilterConfig_t * const filterConfig )
{
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( filterConfig ) );
}


/**
 * \brief Configures the filter (the module is initialized): the filter is ready to start.
 *
 * \param filterConfig [in]: Filter configuration.
 */
static void UtFmac_Ready( const fmac_FilterConfig_t * const filterConfig )
{
    UtFmac_Config( filterConfig );
}


/**
 * \brief Configures and starts the filter (the module is initialized) and clears the logs of the HW model.
 *
 * \param filterConfig [in]: Filter configuration.
 */
static void UtFmac_Running( const fmac_FilterConfig_t * const filterConfig )
{
    UtFmac_Config( filterConfig );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );

    utFmac_X1Cnt    = 0u;
    utFmac_X2Cnt    = 0u;
    utFmac_YCnt     = 0u;
    utFmac_RunCnt   = 0u;
    utFmac_StrayCnt = 0u;
}


/**
 * \brief Returns the value of a buffer configuration register of the given base, size and watermark code.
 *
 * \param base      [in]: Base address of the buffer.
 * \param size      [in]: Size of the buffer.
 * \param watermark [in]: Watermark code of the buffer.
 *
 * \return Register value.
 */
static uint32_t UtFmac_Get_BufCfg( uint32_t base, uint32_t size, uint32_t watermark )
{
    return ( ( base << FMAC_X1BUFCFG_X1_BASE_Pos ) |
             ( size << FMAC_X1BUFCFG_X1_BUF_SIZE_Pos ) |
             ( watermark << FMAC_X1BUFCFG_FULL_WM_Pos ) );
}


/**
 * \brief Checks the value of a buffer configuration register against the base, the size and the watermark code.
 *
 * \param regValue  [in]: Value of the register.
 * \param base      [in]: Expected base address of the buffer.
 * \param size      [in]: Expected size of the buffer.
 * \param watermark [in]: Expected watermark code of the buffer.
 */
static void UtFmac_AssertBufCfg( uint32_t regValue,
                                 uint32_t base,
                                 uint32_t size,
                                 uint32_t watermark )
{
    TEST_ASSERT_EQUAL_HEX32( UtFmac_Get_BufCfg( base, size, watermark ), regValue );
}


/**
 * \brief Returns the value of FMAC_PARAM of a function without the START bit.
 *
 * \param func   [in]: Function code.
 * \param paramP [in]: Parameter P.
 * \param paramQ [in]: Parameter Q.
 * \param paramR [in]: Parameter R.
 *
 * \return Register value.
 */
static uint32_t UtFmac_Get_Param( uint32_t func, uint32_t paramP, uint32_t paramQ, uint32_t paramR )
{
    return ( ( func << FMAC_PARAM_FUNC_Pos ) |
             ( paramR << FMAC_PARAM_R_Pos ) |
             ( paramQ << FMAC_PARAM_Q_Pos ) |
             ( paramP << FMAC_PARAM_P_Pos ) );
}


/**
 * \brief Starts a block request with the test input and the result buffer of the tests.
 *
 * \return Result of Fmac_Set_FilterProcess().
 */
static fmac_RequestState_t UtFmac_StartBlock( void )
{
    return ( Fmac_Set_FilterProcess( utFmac_Input,
                                     UT_FMAC_INPUT_CNT,
                                     &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ],
                                     UT_FMAC_INPUT_CNT ) );
}


/** Fills the result buffer with the pattern (also the guard samples) */
static void UtFmac_FillOutput( void )
{
    for( uint32_t sampleIdx = 0u; UT_FMAC_OUTPUT_MEM_SIZE > sampleIdx; sampleIdx ++ )
    {
        utFmac_OutputMem[ sampleIdx ] = UT_FMAC_FILL_PATTERN;
    }
}


/**
 * \brief Checks the result buffer: the first results are the preset result, the rest and the guard samples keep the
 *        pattern.
 *
 * \param resultCnt [in]: Count of the results written to the buffer.
 */
static void UtFmac_AssertOutput( fmac_SampleCnt_t resultCnt )
{
    for( uint32_t sampleIdx = 0u; UT_FMAC_OUTPUT_MEM_SIZE > sampleIdx; sampleIdx ++ )
    {
        if( ( UT_FMAC_GUARD_SIZE                 >  sampleIdx ) ||
            ( ( UT_FMAC_GUARD_SIZE + resultCnt ) <= sampleIdx )    )
        {
            TEST_ASSERT_EQUAL_INT16( UT_FMAC_FILL_PATTERN, utFmac_OutputMem[ sampleIdx ] );
        }
        else
        {
            TEST_ASSERT_EQUAL_INT16( UT_FMAC_RESULT, utFmac_OutputMem[ sampleIdx ] );
        }
    }
}

/**
 * \brief Records a call of a Dma function and returns its result: the call number \ref utFmac_DmaFailNth of the
 *        function \ref utFmac_DmaFailId fails.
 *
 * \param callId    [in]: Dma function.
 * \param periphId  [in]: Dma controller of the call.
 * \param channelId [in]: Dma stream of the call.
 * \param value     [in]: Value the test checks (transfer mode, memory address, count, ...).
 *
 * \return Result of the Dma function.
 */
static dma_RequestState_t UtFmac_DmaRecord( utFmac_DmaCall_t callId,
                                            uint32_t periphId,
                                            uint32_t channelId,
                                            uint32_t value )
{
    dma_RequestState_t retState = DMA_REQUEST_OK;

    if( UT_FMAC_DMA_LOG_SIZE > utFmac_DmaCnt )
    {
        utFmac_DmaLog[ utFmac_DmaCnt ].Id      = callId;
        utFmac_DmaLog[ utFmac_DmaCnt ].Periph  = periphId;
        utFmac_DmaLog[ utFmac_DmaCnt ].Channel = channelId;
        utFmac_DmaLog[ utFmac_DmaCnt ].Value   = value;
    }
    else
    {
        /* The log is full */
    }

    utFmac_DmaCnt ++;
    utFmac_DmaIdCnt[ callId ] ++;

    if( ( utFmac_DmaFailId  == callId                    ) &&
        ( utFmac_DmaFailNth == utFmac_DmaIdCnt[ callId ] )    )
    {
        retState = DMA_REQUEST_ERROR;
    }
    else
    {
        /* The call succeeds */
    }

    return ( retState );
}


/** Stub of Dma_Init: keeps a copy of the configuration of the first two calls */
static dma_RequestState_t UtFmac_DmaInitStub( dma_ConfigStruct_t * const dmaConfig, int callCnt )
{
    const uint32_t slot = utFmac_DmaIdCnt[ UT_FMAC_DMA_INIT ];

    (void)callCnt;

    if( UT_FMAC_DMA_STREAMS > slot )
    {
        utFmac_DmaInitConfig[ slot ] = *dmaConfig;
    }
    else
    {
        /* Only the two streams of the module are kept */
    }

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_INIT, dmaConfig->DmaPeriphId, dmaConfig->DmaChannel, 0u ) );
}


/** Stub of Dma_Set_TransferErrorIrqActive */
static dma_RequestState_t UtFmac_DmaTeIrqStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_TE_IRQ, dmaBus, dmaChannel, 0u ) );
}


/** Stub of Dma_Set_TransferCompleteIrqActive */
static dma_RequestState_t UtFmac_DmaTcIrqStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_TC_IRQ, dmaBus, dmaChannel, 0u ) );
}


/** Stub of Dma_Set_HalfTransferIrqActive */
static dma_RequestState_t UtFmac_DmaHtIrqStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_HT_IRQ, dmaBus, dmaChannel, 0u ) );
}


/** Stub of Dma_Set_InterruptActive */
static dma_RequestState_t UtFmac_DmaIrqActiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_IRQ_ACTIVE, dmaBus, dmaChannel, 0u ) );
}


/** Stub of Dma_Set_InterruptInactive */
static dma_RequestState_t UtFmac_DmaIrqInactiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_IRQ_INACTIVE, dmaBus, dmaChannel, 0u ) );
}


/** Stub of Dma_Set_TransferMode */
static dma_RequestState_t UtFmac_DmaModeStub( dma_PeriphId_t dmaBus,
                                              dma_ChannelId_t dmaChannel,
                                              dma_TransferMode_t transferMode,
                                              int callCnt )
{
    (void)callCnt;

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_MODE, dmaBus, dmaChannel, (uint32_t)transferMode ) );
}


/** Stub of Dma_Set_MemoryAddr */
static dma_RequestState_t UtFmac_DmaMemAddrStub( dma_PeriphId_t dmaBus,
                                                 dma_ChannelId_t dmaChannel,
                                                 dma_MemoryAddr_t memoryAddr,
                                                 int callCnt )
{
    (void)callCnt;

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_MEM_ADDR, dmaBus, dmaChannel, memoryAddr ) );
}


/** Stub of Dma_Set_DataCount */
static dma_RequestState_t UtFmac_DmaCountStub( dma_PeriphId_t dmaBus,
                                               dma_ChannelId_t dmaChannel,
                                               dma_DataCount_t dataCount,
                                               int callCnt )
{
    (void)callCnt;

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_COUNT, dmaBus, dmaChannel, dataCount ) );
}


/** Stub of Dma_Set_TransferActive */
static dma_RequestState_t UtFmac_DmaXferActiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_XFER_ACTIVE, dmaBus, dmaChannel, 0u ) );
}


/** Stub of Dma_Set_TransferInactive */
static dma_RequestState_t UtFmac_DmaXferInactiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;

    return ( UtFmac_DmaRecord( UT_FMAC_DMA_XFER_INACTIVE, dmaBus, dmaChannel, 0u ) );
}


/** Installs the stubs of the Dma functions the module calls (the default configuration is not recorded) */
static void UtFmac_DmaStubs( void )
{
    Dma_Get_DefaultConfig_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Init_Stub( UtFmac_DmaInitStub );
    Dma_Set_TransferErrorIrqActive_Stub( UtFmac_DmaTeIrqStub );
    Dma_Set_TransferCompleteIrqActive_Stub( UtFmac_DmaTcIrqStub );
    Dma_Set_HalfTransferIrqActive_Stub( UtFmac_DmaHtIrqStub );
    Dma_Set_InterruptActive_Stub( UtFmac_DmaIrqActiveStub );
    Dma_Set_InterruptInactive_Stub( UtFmac_DmaIrqInactiveStub );
    Dma_Set_TransferMode_Stub( UtFmac_DmaModeStub );
    Dma_Set_MemoryAddr_Stub( UtFmac_DmaMemAddrStub );
    Dma_Set_DataCount_Stub( UtFmac_DmaCountStub );
    Dma_Set_TransferActive_Stub( UtFmac_DmaXferActiveStub );
    Dma_Set_TransferInactive_Stub( UtFmac_DmaXferInactiveStub );
}


/** Clears the log of the Dma calls and the counters of the calls (the copies of the Init configurations stay) */
static void UtFmac_ResetDmaLog( void )
{
    utFmac_DmaCnt = 0u;

    for( uint32_t callIdx = 0u; UT_FMAC_DMA_CNT > callIdx; callIdx ++ )
    {
        utFmac_DmaIdCnt[ callIdx ] = 0u;
    }
}


/**
 * \brief Checks one record of the log of the Dma calls.
 *
 * \param logIdx    [in]: Index into the log.
 * \param callId    [in]: Expected Dma function.
 * \param periphId  [in]: Expected controller.
 * \param channelId [in]: Expected stream.
 * \param value     [in]: Expected value of the call.
 */
static void UtFmac_AssertDma( uint32_t logIdx,
                              utFmac_DmaCall_t callId,
                              uint32_t periphId,
                              uint32_t channelId,
                              uint32_t value )
{
    TEST_ASSERT_TRUE( UT_FMAC_DMA_LOG_SIZE > logIdx );
    TEST_ASSERT_EQUAL_UINT32( callId, utFmac_DmaLog[ logIdx ].Id );
    TEST_ASSERT_EQUAL_UINT32( periphId, utFmac_DmaLog[ logIdx ].Periph );
    TEST_ASSERT_EQUAL_UINT32( channelId, utFmac_DmaLog[ logIdx ].Channel );
    TEST_ASSERT_EQUAL_HEX32( value, utFmac_DmaLog[ logIdx ].Value );
}


/**
 * \brief Checks the four Dma calls which arm a stream of DMA1 for a request.
 *
 * \param firstIdx   [in]: Index of the first call in the log.
 * \param channelId  [in]: Stream.
 * \param xferMode   [in]: Expected transfer mode (normal, circular).
 * \param memoryAddr [in]: Expected memory address.
 * \param sampleCnt  [in]: Expected count of the samples.
 */
static void UtFmac_AssertArm( uint32_t firstIdx,
                              dma_ChannelId_t channelId,
                              dma_TransferMode_t xferMode,
                              uint32_t memoryAddr,
                              uint32_t sampleCnt )
{
    uint32_t logIdx = firstIdx;

    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_MODE, DMA_PERIPH_1, channelId, (uint32_t)xferMode );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_MEM_ADDR, DMA_PERIPH_1, channelId, memoryAddr );
    UtFmac_AssertDma( logIdx ++, UT_FMAC_DMA_COUNT, DMA_PERIPH_1, channelId, sampleCnt );
    UtFmac_AssertDma( logIdx, UT_FMAC_DMA_XFER_ACTIVE, DMA_PERIPH_1, channelId, 0u );
}


/**
 * \brief Checks the configuration the module passed to Dma_Init for a stream: 16-bit data, the direct mode (the same
 *        size on both sides), the normal mode, fixed peripheral and incrementing memory address, the placeholder
 *        transfer, the transfer error callback.
 *
 * \param slot       [in]: 0 for the stream of the input data, 1 for the stream of the results.
 * \param channelId  [in]: Expected stream.
 * \param direction  [in]: Expected direction.
 * \param request    [in]: Expected DMAMUX1 request of the FMAC.
 * \param periphAddr [in]: Expected peripheral address (FMAC_WDATA, FMAC_RDATA).
 */
static void UtFmac_AssertInitStream( uint32_t slot,
                                     dma_ChannelId_t channelId,
                                     dma_Direction_t direction,
                                     dma_PeriphReqId_t request,
                                     uint32_t periphAddr )
{
    const dma_ConfigStruct_t * const dmaConfig = &utFmac_DmaInitConfig[ slot ];

    TEST_ASSERT_EQUAL( channelId, dmaConfig->DmaChannel );
    TEST_ASSERT_EQUAL( direction, dmaConfig->Direction );
    TEST_ASSERT_EQUAL( request, dmaConfig->PeripheralReqId );
    TEST_ASSERT_EQUAL_HEX32( periphAddr, dmaConfig->PeriphAddress );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL, dmaConfig->TransferMode );
    TEST_ASSERT_EQUAL( DMA_PERIPH_ADDR_STATIC, dmaConfig->PeriphAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT, dmaConfig->MemoryAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, dmaConfig->PeriphTransferSize );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, dmaConfig->MemoryTransferSize );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_PLACEHOLDER_CNT, dmaConfig->DataCount );
    TEST_ASSERT_NOT_EQUAL( 0u, dmaConfig->MemoryAddress );
    TEST_ASSERT_NOT_NULL( dmaConfig->TransferErrorCallback );
}


/** Activates the HW model and initializes the module in DMA mode, the log of the Dma calls is cleared */
static void UtFmac_InitDma( void )
{
    UtFmac_Init( FMAC_XFER_MODE_DMA );
    UtFmac_ResetDmaLog();
}


/**
 * \brief Returns the stream request of the tests: the test input and the result buffer, 6 samples.
 *
 * \param bufferMode [in]: Buffer mode of the stream.
 *
 * \return Stream request.
 */
static fmac_StreamRequest_t UtFmac_Get_StreamRequest( fmac_BufferMode_t bufferMode )
{
    fmac_StreamRequest_t streamRequest =
    {
        .Input      = utFmac_Input,
        .InputSize  = UT_FMAC_INPUT_CNT,
        .Output     = &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ],
        .OutputSize = UT_FMAC_INPUT_CNT,
        .BufferMode = bufferMode,
    };

    return ( streamRequest );
}


/**
 * \brief Starts a stream with the test input and the result buffer of the tests.
 *
 * \param bufferMode [in]: Buffer mode of the stream.
 *
 * \return Result of Fmac_Set_StreamStart().
 */
static fmac_RequestState_t UtFmac_StartStream( fmac_BufferMode_t bufferMode )
{
    const fmac_StreamRequest_t streamRequest = UtFmac_Get_StreamRequest( bufferMode );

    return ( Fmac_Set_StreamStart( &streamRequest ) );
}


/**
 * \brief Starts a dot product request with the 4 test inputs and a result buffer of one sample.
 *
 * \return Result of Fmac_Set_FilterProcess().
 */
static fmac_RequestState_t UtFmac_StartDotProduct( void )
{
    return ( Fmac_Set_FilterProcess( utFmac_Input, UT_FMAC_TAPS, &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ], 1u ) );
}


/**
 * \brief Returns the configuration of the FIR test filter with the function of a vector function.
 *
 * \param function [in]: Vector function.
 *
 * \return Filter configuration.
 */
static fmac_FilterConfig_t UtFmac_Get_VectorConfig( fmac_Function_t function )
{
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();

    filterConfig.Function = function;

    return ( filterConfig );
}


/**
 * \brief Runs two block requests of a convolution or a correlation (POLL mode) and checks the zero state, the
 *        coefficients, the function started and the data moved by Fmac_Task().
 *
 * \param function [in]: Convolution or correlation.
 */
static void UtFmac_AssertVectorRequests( fmac_Function_t function )
{
    fmac_FilterConfig_t   filterConfig = UtFmac_Get_VectorConfig( function );
    const fmac_Sample_t * expectedCoef = utFmac_CoefB;
    fmac_XferState_t      xferState    = FMAC_XFER_STATE_BUSY;

    if( FMAC_FUNC_CORRELATION == function )
    {
        expectedCoef = utFmac_CoefBRev;
    }
    else
    {
        /* The convolution keeps the order of the coefficients */
    }

    UtFmac_Init( FMAC_XFER_MODE_POLL );
    UtFmac_Running( &filterConfig );
    FMAC->RDATA = UT_FMAC_RESULT_RAW;

    for( uint32_t requestIdx = 0u; UT_FMAC_STREAM_LAPS > requestIdx; requestIdx ++ )
    {
        utFmac_ResetCnt = 0u;
        utFmac_StartCnt = 0u;
        utFmac_X1Cnt    = 0u;
        utFmac_X2Cnt    = 0u;
        utFmac_RunCnt   = 0u;
        UtFmac_FillOutput();

        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

        TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ResetCnt );
        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_VECTOR_STARTS, utFmac_StartCnt );
        TEST_ASSERT_EQUAL_UINT32( FMAC_REG_FUNC_LOAD_X2, utFmac_StartLog[ UT_FMAC_LOAD_X2_IDX ].Func );
        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS, utFmac_StartLog[ UT_FMAC_LOAD_X2_IDX ].ParamP );
        TEST_ASSERT_EQUAL_INT16_ARRAY( expectedCoef, utFmac_X2Data, UT_FMAC_TAPS );
        TEST_ASSERT_EQUAL_UINT32( FMAC_REG_FUNC_LOAD_X1, utFmac_StartLog[ UT_FMAC_LOAD_X1_IDX ].Func );
        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS - 1u, utFmac_StartLog[ UT_FMAC_LOAD_X1_IDX ].ParamP );
        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS - 1u, utFmac_X1Cnt );
        TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_ZeroState, utFmac_X1Data, UT_FMAC_TAPS - 1u );
        TEST_ASSERT_EQUAL_UINT32( FMAC_REG_FUNC_FIR, utFmac_StartLog[ UT_FMAC_RUN_IDX ].Func );
        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TAPS, utFmac_StartLog[ UT_FMAC_RUN_IDX ].ParamP );
        TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StartLog[ UT_FMAC_RUN_IDX ].ParamQ );
        TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );

        FMAC->SR = UT_FMAC_REG_ZERO;
        Fmac_Task();

        TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_CNT, utFmac_RunCnt );
        TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_Input, utFmac_RunData, UT_FMAC_INPUT_CNT );
        UtFmac_AssertOutput( UT_FMAC_INPUT_CNT );
        TEST_ASSERT_EQUAL_UINT32( requestIdx + 1u, utFmac_HalfCnt );
        TEST_ASSERT_EQUAL_UINT32( requestIdx + 1u, utFmac_CompleteCnt );
        TEST_ASSERT_BITS_LOW( FMAC_PARAM_START, FMAC->PARAM );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
        TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
        TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_StrayCnt );
    }
}

#endif /* FMAC */
