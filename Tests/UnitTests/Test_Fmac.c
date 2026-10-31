/**
 * \author Mr.Nobody
 * \file Test_Fmac.c
 * \ingroup Fmac
 * \brief Unit tests of the filter math accelerator (FMAC) module.
 *
 * Fmac.c and Fmac_Reg.c are compiled unchanged, the FMAC registers are emulated by RegMem, the RCC, NVIC and
 * Gpdma modules are mocked by CMock.
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
 * \note The Gpdma functions the module calls are replaced by stubs (see UtFmac_GpdmaStubs) which record every
 *       call (function, controller, channel and the value the test checks: block size, address, link register) in a
 *       log, keep a copy of the configuration of both channels (so that the tests can call the callbacks registered by
 *       the module) and let a chosen call fail. The tests prove which transfers the module arms and how it reacts to
 *       the callbacks; the transfers themselves are executed by the integration tests on hardware.
 *
 * \note The FMAC does not exist on STM32H503, H523, H533, H543 and H553. The test runner is generated from this
 *       file without running the preprocessor, so every test case function below is defined unconditionally
 *       (the same list on all 12 device lines); on the lines without the peripheral the test cases use
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
#include "MockGpdma_Port.h"                 /* Gpdma module mock              */
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

/** Gpdma functions recorded by the stubs */
typedef enum
{
    UT_FMAC_GPDMA_INIT = 0u,    /**< Gpdma_Init                    */
    UT_FMAC_GPDMA_DEINIT,       /**< Gpdma_Deinit                  */
    UT_FMAC_GPDMA_IRQ_ACTIVE,   /**< Gpdma_Set_InterruptActive     */
    UT_FMAC_GPDMA_LIST_CONFIG,  /**< Gpdma_Get_XferListConfig      */
    UT_FMAC_GPDMA_LINK_REG,     /**< Gpdma_Get_XferList_LinkReg    */
    UT_FMAC_GPDMA_LIST_LINK,    /**< Gpdma_Set_XferListLink        */
    UT_FMAC_GPDMA_BLOCK_SIZE,   /**< Gpdma_Set_BlockSize           */
    UT_FMAC_GPDMA_SRC_ADDR,     /**< Gpdma_Set_SourceAddr          */
    UT_FMAC_GPDMA_DST_ADDR,     /**< Gpdma_Set_DestinationAddr     */
    UT_FMAC_GPDMA_CH_ACTIVE,    /**< Gpdma_Set_ChannelActive       */
    UT_FMAC_GPDMA_CH_INACTIVE,  /**< Gpdma_Set_ChannelInactive     */
    UT_FMAC_GPDMA_NONE,         /**< No call (no failure injected) */
    UT_FMAC_GPDMA_CNT           /**< Count of the items            */
}   utFmac_GpdmaCall_t;

/** Record of one Gpdma call */
typedef struct
{
    utFmac_GpdmaCall_t Id;      /**< Gpdma function                        */
    uint32_t           Periph;  /**< Controller of the call                */
    uint32_t           Channel; /**< Channel of the call                   */
    uint32_t           Value;   /**< Value of the call the test checks     */
}   utFmac_GpdmaRecord_t;

/** Gpdma call which fails in a test of a failing arming: the function and the number of its call */
typedef struct
{
    utFmac_GpdmaCall_t Id;      /**< Gpdma function                        */
    uint32_t           Nth;     /**< Number of the call which fails        */
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
static gpdma_RequestState_t UtFmac_GpdmaRecord     ( utFmac_GpdmaCall_t callId,
                                                     uint32_t periphId,
                                                     uint32_t channelId,
                                                     uint32_t value );
static void                UtFmac_GpdmaStubs       ( void );
static void                UtFmac_ResetGpdmaLog    ( void );
static void                UtFmac_AssertGpdma      ( uint32_t logIdx,
                                                     utFmac_GpdmaCall_t callId,
                                                     uint32_t periphId,
                                                     uint32_t channelId,
                                                     uint32_t value );
static void                UtFmac_AssertArm        ( uint32_t firstIdx,
                                                     gpdma_ChannelId_t channelId,
                                                     uint32_t blockSize,
                                                     uint32_t srcAddr,
                                                     uint32_t dstAddr );
static void                UtFmac_AssertXfer       ( uint32_t slot,
                                                     gpdma_Direction_t direction,
                                                     gpdma_PeriphReqId_t request,
                                                     gpdma_XferListExecMode_t execMode );
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

/** Count of two events (calls, errors, channels stopped) and the indexes of the third to the fifth Gpdma call */
#define UT_FMAC_TWICE                       ( 2u )
#define UT_FMAC_THIRD                       ( 2u )
#define UT_FMAC_FOURTH                      ( 3u )
#define UT_FMAC_FIFTH                       ( 4u )

/** Count of the laps of a stream and of the repeated requests of a test */
#define UT_FMAC_STREAM_LAPS                 ( 2u )

/** Count of the Gpdma channels of the module and size of the log of the Gpdma calls */
#define UT_FMAC_DMA_STREAMS                 ( 2u )
#define UT_FMAC_DMA_LOG_SIZE                ( 96u )

/** Argument of a Gpdma call which has no such argument, and the link register the Gpdma stub returns */
#define UT_FMAC_NA                          ( 0xFFFFu )
#define UT_FMAC_LINKREG_MARK                ( 0xA5A5A5A5u )

/** Size of a sample in bytes and of the 6 test samples, address of a buffer as the Gpdma takes it */
#define UT_FMAC_BYTES_PER_SAMPLE            ( 2u )
#define UT_FMAC_INPUT_BYTES                 ( UT_FMAC_INPUT_CNT * UT_FMAC_BYTES_PER_SAMPLE )
#define UT_FMAC_ADDR( ptr )                 ( (uint32_t)(uintptr_t)( ptr ) )

/** Count of the Gpdma calls which arm a channel and index of the block size call of the first channel */
#define UT_FMAC_ARM_CALLS                   ( 7u )
#define UT_FMAC_BLOCK_SIZE_IDX              ( 3u )

/** Count of the Gpdma calls of the initialization, of the failed initialization and of the deinitialization */
#define UT_FMAC_INIT_GPDMA_CALLS            ( 4u )
#define UT_FMAC_INIT_FAIL_GPDMA_CALLS       ( 5u )
#define UT_FMAC_DEINIT_GPDMA_CALLS          ( 4u )

/** Count of the arming steps tested to fail */
#define UT_FMAC_ARM_FAIL_CNT                ( 8u )

/** Functions started by a vector request (X2 load, X1 load, filter function) and the index of the filter function */
#define UT_FMAC_VECTOR_STARTS               ( 3u )
#define UT_FMAC_RUN_IDX                     ( 2u )

/** Gpdma errors the module enables, and the error interrupt enables of FMAC_CR */
#define UT_FMAC_DMA_ERROR_MASK              ( 0x0Fu )
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

/** The model clears START of a filter function at once (a request read out by the interrupt or by the Gpdma ends) */
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

/** Log of the Gpdma calls, their count and the count per function */
static utFmac_GpdmaRecord_t utFmac_GpdmaLog[ UT_FMAC_DMA_LOG_SIZE ];
static uint32_t             utFmac_GpdmaCnt;
static uint32_t             utFmac_GpdmaIdCnt[ UT_FMAC_GPDMA_CNT ];

/** Gpdma function and number of its call which fails (number 0: none) */
static utFmac_GpdmaCall_t utFmac_GpdmaFailId;
static uint32_t           utFmac_GpdmaFailNth;

/** Copies of the configurations passed to Gpdma_Init and of the transfers they point to (input data, results) */
static gpdma_ConfigStruct_t   utFmac_DmaInitConfig[ UT_FMAC_DMA_STREAMS ];
static gpdma_TransferConfig_t utFmac_DmaInitXfer[ UT_FMAC_DMA_STREAMS ];

/** Copies of the transfers passed to Gpdma_Get_XferListConfig by the last request (results, input data) */
static gpdma_TransferConfig_t utFmac_DmaXfer[ UT_FMAC_DMA_STREAMS ];

/** Steps of the arming of a channel which fail in turn: the Gpdma function and the number of its call */
static const utFmac_ArmFail_t utFmac_ArmFailLut[ UT_FMAC_ARM_FAIL_CNT ] =
{
    { UT_FMAC_GPDMA_LIST_CONFIG, 1u },
    { UT_FMAC_GPDMA_LINK_REG,    1u },
    { UT_FMAC_GPDMA_LIST_LINK,   1u },
    { UT_FMAC_GPDMA_BLOCK_SIZE,  1u },
    { UT_FMAC_GPDMA_SRC_ADDR,    1u },
    { UT_FMAC_GPDMA_DST_ADDR,    1u },
    { UT_FMAC_GPDMA_CH_ACTIVE,   1u },
    { UT_FMAC_GPDMA_CH_ACTIVE,   UT_FMAC_TWICE },
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
    utFmac_GpdmaFailId       = UT_FMAC_GPDMA_NONE;
    utFmac_GpdmaFailNth      = 0u;

    UtFmac_ResetGpdmaLog();
    UtFmac_GpdmaStubs();
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
 * - FMAC_REQUEST_OK, POLL transfer mode, Gpdma1 channel 0 (input data) and channel 1 (results) low priority,
 *   IRQ priority 0, no callbacks.
 * - NULL pointer: FMAC_REQUEST_ERROR.
 */
void Ut_Fmac_Get_DefaultConfig_ReturnsDefaults( void )
{
#if defined(FMAC)
    fmac_Config_t config;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( FMAC_XFER_MODE_POLL, config.XferMode );
    TEST_ASSERT_EQUAL( GPDMA_PERIPH_1, config.DmaPeriphId );
    TEST_ASSERT_EQUAL( GPDMA_CHANNEL_0, config.DmaWriteChannelId );
    TEST_ASSERT_EQUAL( GPDMA_CHANNEL_1, config.DmaReadChannelId );
    TEST_ASSERT_EQUAL( GPDMA_PRIORITY_LOW, config.DmaPriority );
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
 *   running (a vector function of a request which is read out by the interrupt or by the Gpdma at once is stopped by
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
 * \brief   Fmac_Init() in DMA mode initializes the two Gpdma channels and enables the interrupt in Nvic.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK; Gpdma calls in this order: init of the channel of the input data (channel 0), its interrupt,
 *   init of the channel of the results (channel 1), its interrupt.
 * - Both channels: controller 1, low priority, one transfer, an error callback and the error mask of the transfer,
 *   configuration, update and trigger overrun errors; the channel of the input data has no other callback, the
 *   channel of the results has the complete and the half callback.
 * - Nvic: handler, priority and enable of the FMAC interrupt; FMAC_CR is zero.
 */
void Ut_Fmac_Init_Dma_InitializesGpdmaChannels( void )
{
#if defined(FMAC)
    UtFmac_Init( FMAC_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INIT_GPDMA_CALLS, utFmac_GpdmaCnt );
    UtFmac_AssertGpdma( 0u, UT_FMAC_GPDMA_INIT, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    UtFmac_AssertGpdma( 1u, UT_FMAC_GPDMA_IRQ_ACTIVE, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    UtFmac_AssertGpdma( UT_FMAC_THIRD, UT_FMAC_GPDMA_INIT, GPDMA_PERIPH_1, GPDMA_CHANNEL_1, 0u );
    UtFmac_AssertGpdma( UT_FMAC_FOURTH, UT_FMAC_GPDMA_IRQ_ACTIVE, GPDMA_PERIPH_1, GPDMA_CHANNEL_1, 0u );

    for( uint32_t streamIdx = 0u; UT_FMAC_DMA_STREAMS > streamIdx; streamIdx ++ )
    {
        TEST_ASSERT_EQUAL( GPDMA_PERIPH_1, utFmac_DmaInitConfig[ streamIdx ].PeriphId );
        TEST_ASSERT_EQUAL( GPDMA_PRIORITY_LOW, utFmac_DmaInitConfig[ streamIdx ].ChannelPrio );
        TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_DmaInitConfig[ streamIdx ].TransfersCount );
        TEST_ASSERT_NOT_NULL( utFmac_DmaInitConfig[ streamIdx ].ErrorIsr );
        TEST_ASSERT_EQUAL_HEX32( UT_FMAC_DMA_ERROR_MASK, utFmac_DmaInitConfig[ streamIdx ].ErrorMask );
    }

    TEST_ASSERT_EQUAL( GPDMA_CHANNEL_0, utFmac_DmaInitConfig[ 0u ].ChannelId );
    TEST_ASSERT_NULL( utFmac_DmaInitConfig[ 0u ].TransferCompleteIsr );
    TEST_ASSERT_NULL( utFmac_DmaInitConfig[ 0u ].HalfTransferIsr );
    TEST_ASSERT_EQUAL( GPDMA_CHANNEL_1, utFmac_DmaInitConfig[ 1u ].ChannelId );
    TEST_ASSERT_NOT_NULL( utFmac_DmaInitConfig[ 1u ].TransferCompleteIsr );
    TEST_ASSERT_NOT_NULL( utFmac_DmaInitConfig[ 1u ].HalfTransferIsr );

    TEST_ASSERT_NOT_NULL( utFmac_Isr );
    TEST_ASSERT_EQUAL_HEX32( UT_FMAC_REG_ZERO, FMAC->CR );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() in DMA mode uses the channels of the configuration and registers a half transfer callback only
 *          for a configured half callback.
 *
 * \par Expected results
 * - Channels 3 (input data) and 4 (results) of Gpdma controller 2 with high priority are initialized.
 * - Without a half callback in the configuration the channel of the results has no half transfer callback.
 */
void Ut_Fmac_Init_Dma_ConfiguredChannels_AreUsed( void )
{
#if defined(FMAC)
    fmac_Config_t config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );

    config.DmaPeriphId       = GPDMA_PERIPH_2;
    config.DmaWriteChannelId = GPDMA_CHANNEL_3;
    config.DmaReadChannelId  = GPDMA_CHANNEL_4;
    config.DmaPriority       = GPDMA_PRIORITY_HIGH;
    config.XferHalfCallback  = NULL;

    UtFmac_ModelStart();
    UtFmac_Expect_Init( FMAC_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Init( &config ) );

    UtFmac_AssertGpdma( 0u, UT_FMAC_GPDMA_INIT, GPDMA_PERIPH_2, GPDMA_CHANNEL_3, 0u );
    UtFmac_AssertGpdma( 1u, UT_FMAC_GPDMA_IRQ_ACTIVE, GPDMA_PERIPH_2, GPDMA_CHANNEL_3, 0u );
    UtFmac_AssertGpdma( UT_FMAC_THIRD, UT_FMAC_GPDMA_INIT, GPDMA_PERIPH_2, GPDMA_CHANNEL_4, 0u );
    UtFmac_AssertGpdma( UT_FMAC_FOURTH, UT_FMAC_GPDMA_IRQ_ACTIVE, GPDMA_PERIPH_2, GPDMA_CHANNEL_4, 0u );
    TEST_ASSERT_EQUAL( GPDMA_PRIORITY_HIGH, utFmac_DmaInitConfig[ 0u ].ChannelPrio );
    TEST_ASSERT_EQUAL( GPDMA_PRIORITY_HIGH, utFmac_DmaInitConfig[ 1u ].ChannelPrio );
    TEST_ASSERT_NOT_NULL( utFmac_DmaInitConfig[ 1u ].TransferCompleteIsr );
    TEST_ASSERT_NULL( utFmac_DmaInitConfig[ 1u ].HalfTransferIsr );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() rejects an invalid DMA configuration.
 *
 * \par Expected results
 * - FMAC_REQUEST_ERROR without any Rcc / Nvic / Gpdma call for: the same channel for both streams, a channel out of
 *   the list (input data and results), a Gpdma controller out of the list, a priority out of the list.
 */
void Ut_Fmac_Init_Dma_InvalidConfig_ReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );

    config.DmaReadChannelId = config.DmaWriteChannelId;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    config.DmaWriteChannelId = GPDMA_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    config.DmaReadChannelId = GPDMA_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    config.DmaPeriphId = GPDMA_PERIPH_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    config = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    config.DmaPriority = GPDMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_GpdmaCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_ResetCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() in DMA mode deinitializes the module when Gpdma rejects the second channel.
 *
 * \par Expected results
 * - Gpdma_Init() of the second channel fails: FMAC_REQUEST_ERROR, the Nvic interrupt is disabled, the channel which
 *   was initialized is stopped and released (only that one), the unit is reset through Rcc and the clock is disabled,
 *   the module is not initialized.
 */
void Ut_Fmac_Init_Dma_GpdmaInitFails_DeinitializesAndReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t      config      = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    fmac_FilterState_t filterState = FMAC_FILTER_READY;

    UtFmac_ModelStart();
    utFmac_GpdmaFailId  = UT_FMAC_GPDMA_INIT;
    utFmac_GpdmaFailNth = UT_FMAC_TWICE;
    UtFmac_Expect_Init( FMAC_XFER_MODE_DMA );
    UtFmac_Expect_Release( FMAC_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INIT_FAIL_GPDMA_CALLS, utFmac_GpdmaCnt );
    UtFmac_AssertGpdma( 0u, UT_FMAC_GPDMA_INIT, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    UtFmac_AssertGpdma( 1u, UT_FMAC_GPDMA_IRQ_ACTIVE, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    UtFmac_AssertGpdma( UT_FMAC_THIRD, UT_FMAC_GPDMA_INIT, GPDMA_PERIPH_1, GPDMA_CHANNEL_1, 0u );
    UtFmac_AssertGpdma( UT_FMAC_FOURTH, UT_FMAC_GPDMA_CH_INACTIVE, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    UtFmac_AssertGpdma( UT_FMAC_FIFTH, UT_FMAC_GPDMA_DEINIT, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Init() in DMA mode deinitializes the module when Gpdma can not enable the channel interrupt.
 *
 * \par Expected results
 * - Gpdma_Set_InterruptActive() of the first channel fails: FMAC_REQUEST_ERROR, the channel is stopped and released,
 *   the second channel is not initialized, the module is not initialized.
 */
void Ut_Fmac_Init_Dma_GpdmaInterruptFails_DeinitializesAndReturnsError( void )
{
#if defined(FMAC)
    fmac_Config_t      config      = UtFmac_Get_Config( FMAC_XFER_MODE_DMA );
    fmac_FilterState_t filterState = FMAC_FILTER_READY;

    UtFmac_ModelStart();
    utFmac_GpdmaFailId  = UT_FMAC_GPDMA_IRQ_ACTIVE;
    utFmac_GpdmaFailNth = 1u;
    UtFmac_Expect_Init( FMAC_XFER_MODE_DMA );
    UtFmac_Expect_Release( FMAC_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_GpdmaIdCnt[ UT_FMAC_GPDMA_INIT ] );
    UtFmac_AssertGpdma( UT_FMAC_THIRD, UT_FMAC_GPDMA_CH_INACTIVE, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    UtFmac_AssertGpdma( UT_FMAC_FOURTH, UT_FMAC_GPDMA_DEINIT, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Get_FilterState( &filterState ) );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Deinit() in DMA mode stops and releases both Gpdma channels.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK; Gpdma calls in this order: both channels inactive, both channels released.
 * - The interrupt is disabled in Nvic, the unit is reset and the clock disabled through Rcc.
 */
void Ut_Fmac_Deinit_Dma_ReleasesGpdmaChannels( void )
{
#if defined(FMAC)
    UtFmac_InitDma();

    UtFmac_Expect_Release( FMAC_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_DEINIT_GPDMA_CALLS, utFmac_GpdmaCnt );
    UtFmac_AssertGpdma( 0u, UT_FMAC_GPDMA_CH_INACTIVE, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    UtFmac_AssertGpdma( 1u, UT_FMAC_GPDMA_CH_INACTIVE, GPDMA_PERIPH_1, GPDMA_CHANNEL_1, 0u );
    UtFmac_AssertGpdma( UT_FMAC_THIRD, UT_FMAC_GPDMA_DEINIT, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    UtFmac_AssertGpdma( UT_FMAC_FOURTH, UT_FMAC_GPDMA_DEINIT, GPDMA_PERIPH_1, GPDMA_CHANNEL_1, 0u );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Deinit() in DMA mode returns an error when Gpdma can not release a channel, the module is released.
 *
 * \par Expected results
 * - Gpdma_Deinit() of the first channel fails: FMAC_REQUEST_ERROR, the second channel is released anyway, the unit
 *   is released through Rcc, every function returns FMAC_REQUEST_ERROR afterwards.
 */
void Ut_Fmac_Deinit_Dma_GpdmaDeinitFails_ReturnsErrorAndReleases( void )
{
#if defined(FMAC)
    fmac_FilterState_t filterState = FMAC_FILTER_READY;

    UtFmac_InitDma();
    utFmac_GpdmaFailId  = UT_FMAC_GPDMA_DEINIT;
    utFmac_GpdmaFailNth = 1u;

    UtFmac_Expect_Release( FMAC_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Deinit() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_GpdmaIdCnt[ UT_FMAC_GPDMA_DEINIT ] );
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
 * \brief   A block request in DMA mode arms the two Gpdma channels, the CPU moves no sample.
 *
 * \par Expected results
 * - Fmac_Set_FilterProcess() returns FMAC_REQUEST_OK; for the channel of the results and then for the channel of
 *   the input data: transfer list node, link register, block size of 12 bytes (6 samples of 16 bits), source,
 *   destination, channel enable - in this order.
 * - Results: peripheral to memory, request FMAC read, FMAC_RDATA (fixed) to the output buffer (incrementing); input
 *   data: memory to peripheral, request FMAC write, input buffer (incrementing) to FMAC_WDATA (fixed); both 16 bit,
 *   one request per sample, executed once.
 * - FMAC_CR keeps WIEN and RIEN clear, Fmac_Task() moves nothing, the state is busy.
 */
void Ut_Fmac_Set_FilterProcess_Dma_ArmsGpdmaChannels( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    UtFmac_ResetGpdmaLog();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_DMA_STREAMS * UT_FMAC_ARM_CALLS, utFmac_GpdmaCnt );
    UtFmac_AssertArm( 0u,
                      GPDMA_CHANNEL_1,
                      UT_FMAC_INPUT_BYTES,
                      UT_FMAC_ADDR( &FMAC->RDATA ),
                      UT_FMAC_ADDR( &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ] ) );
    UtFmac_AssertArm( UT_FMAC_ARM_CALLS,
                      GPDMA_CHANNEL_0,
                      UT_FMAC_INPUT_BYTES,
                      UT_FMAC_ADDR( utFmac_Input ),
                      UT_FMAC_ADDR( &FMAC->WDATA ) );

    UtFmac_AssertXfer( 0u, GPDMA_DIR_PERIPH_TO_MEMORY, GPDMA_REQ_FMAC_READ, GPDMA_XFER_LIST_EXEC_ONCE );
    UtFmac_AssertXfer( 1u, GPDMA_DIR_MEMORY_TO_PERIPH, GPDMA_REQ_FMAC_WRITE, GPDMA_XFER_LIST_EXEC_ONCE );

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
 * \brief   The Gpdma callbacks of the results end a block request in DMA mode.
 *
 * \par Expected results
 * - The half transfer callback calls the half callback once, the request goes on.
 * - The transfer complete callback calls the complete callback once, the state is idle, the filter keeps running.
 * - A further transfer complete callback without a request does nothing.
 */
void Ut_Fmac_Set_FilterProcess_Dma_GpdmaCallbacksEndRequest( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_STOPPED;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    utFmac_DmaInitConfig[ 1u ].HalfTransferIsr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );

    utFmac_DmaInitConfig[ 1u ].TransferCompleteIsr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_RUNNING, filterState );

    utFmac_DmaInitConfig[ 1u ].TransferCompleteIsr();
    utFmac_DmaInitConfig[ 1u ].HalfTransferIsr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_HalfCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A request of the largest size a Gpdma block takes is accepted, a larger one is rejected in DMA mode.
 *
 * \par Expected results
 * - FMAC_DMA_SAMPLE_CNT_MAX + 1 samples: FMAC_REQUEST_ERROR, no Gpdma call, the stream request too.
 * - FMAC_DMA_SAMPLE_CNT_MAX samples: FMAC_REQUEST_OK, the block size of both channels is 2 bytes per sample.
 */
void Ut_Fmac_Set_FilterProcess_Dma_SizeLimit_IsChecked( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t  filterConfig  = UtFmac_Get_FirConfig();
    fmac_StreamRequest_t streamRequest = UtFmac_Get_StreamRequest( FMAC_BUFFER_ONE_SHOT );
    fmac_Sample_t * const output       = &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ];

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    UtFmac_ResetGpdmaLog();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR,
                       Fmac_Set_FilterProcess( utFmac_Input,
                                               FMAC_DMA_SAMPLE_CNT_MAX + 1u,
                                               output,
                                               FMAC_DMA_SAMPLE_CNT_MAX + 1u ) );
    streamRequest.InputSize  = FMAC_DMA_SAMPLE_CNT_MAX + 1u;
    streamRequest.OutputSize = FMAC_DMA_SAMPLE_CNT_MAX + 1u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStart( &streamRequest ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_GpdmaCnt );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK,
                       Fmac_Set_FilterProcess( utFmac_Input,
                                               FMAC_DMA_SAMPLE_CNT_MAX,
                                               output,
                                               FMAC_DMA_SAMPLE_CNT_MAX ) );
    UtFmac_AssertGpdma( UT_FMAC_BLOCK_SIZE_IDX,
                        UT_FMAC_GPDMA_BLOCK_SIZE,
                        GPDMA_PERIPH_1,
                        GPDMA_CHANNEL_1,
                        FMAC_DMA_SAMPLE_CNT_MAX * UT_FMAC_BYTES_PER_SAMPLE );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A circular stream in DMA mode uses cyclic transfer lists and reports every lap.
 *
 * \par Expected results
 * - Fmac_Set_StreamStart(): both channels are armed with the cyclic execution of the transfer list, 12 bytes.
 * - Every transfer complete callback of the results calls the complete callback, the state stays busy; the half
 *   transfer callback calls the half callback.
 * - Fmac_Set_StreamStop() stops both Gpdma channels, clears the DMA request enables of FMAC_CR, the state is idle, the
 *   filter is FMAC_FILTER_STOPPED.
 */
void Ut_Fmac_Set_StreamStart_CircularDma_UsesCyclicLists( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    UtFmac_ResetGpdmaLog();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );
    UtFmac_AssertXfer( 0u, GPDMA_DIR_PERIPH_TO_MEMORY, GPDMA_REQ_FMAC_READ, GPDMA_XFER_LIST_EXEC_CYCLIC_ALL );
    UtFmac_AssertXfer( 1u, GPDMA_DIR_MEMORY_TO_PERIPH, GPDMA_REQ_FMAC_WRITE, GPDMA_XFER_LIST_EXEC_CYCLIC_ALL );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_BYTES, utFmac_DmaXfer[ 0u ].BlockSize );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_INPUT_BYTES, utFmac_DmaXfer[ 1u ].BlockSize );

    for( uint32_t lapIdx = 0u; UT_FMAC_STREAM_LAPS > lapIdx; lapIdx ++ )
    {
        utFmac_DmaInitConfig[ 1u ].HalfTransferIsr();
        utFmac_DmaInitConfig[ 1u ].TransferCompleteIsr();

        TEST_ASSERT_EQUAL_UINT32( lapIdx + 1u, utFmac_HalfCnt );
        TEST_ASSERT_EQUAL_UINT32( lapIdx + 1u, utFmac_CompleteCnt );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
        TEST_ASSERT_EQUAL( FMAC_XFER_STATE_BUSY, xferState );
    }

    UtFmac_ResetGpdmaLog();
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_StreamStop() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_GpdmaCnt );
    UtFmac_AssertGpdma( 0u, UT_FMAC_GPDMA_CH_INACTIVE, GPDMA_PERIPH_1, GPDMA_CHANNEL_0, 0u );
    UtFmac_AssertGpdma( 1u, UT_FMAC_GPDMA_CH_INACTIVE, GPDMA_PERIPH_1, GPDMA_CHANNEL_1, 0u );
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
 * \brief   Fmac_Set_StreamStop() in DMA mode returns an error when Gpdma can not stop a channel, the stream is ended.
 *
 * \par Expected results
 * - Gpdma_Set_ChannelInactive() of the first channel fails: FMAC_REQUEST_ERROR, the second channel is stopped
 *   anyway, the state is idle and the filter is FMAC_FILTER_STOPPED.
 */
void Ut_Fmac_Set_StreamStop_Dma_GpdmaStopFails_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartStream( FMAC_BUFFER_CIRCULAR ) );

    UtFmac_ResetGpdmaLog();
    utFmac_GpdmaFailId  = UT_FMAC_GPDMA_CH_INACTIVE;
    utFmac_GpdmaFailNth = 1u;
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStop() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_GpdmaIdCnt[ UT_FMAC_GPDMA_CH_INACTIVE ] );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   Fmac_Set_FilterStop() in DMA mode stops the Gpdma channels of a request in progress.
 *
 * \par Expected results
 * - FMAC_REQUEST_OK, both channels are stopped, no callback is called, the state is idle, the DMA request enables of
 *   FMAC_CR are cleared and the filter is FMAC_FILTER_STOPPED.
 */
void Ut_Fmac_Set_FilterStop_Dma_RequestInProgress_StopsChannels( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_BUSY;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    UtFmac_ResetGpdmaLog();
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStop() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_GpdmaIdCnt[ UT_FMAC_GPDMA_CH_INACTIVE ] );
    TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A Gpdma error ends a request in DMA mode with FMAC_XFER_ERROR_DMA.
 *
 * \par Expected results
 * - The error callback of Gpdma is called during a block request: the error callback of the module is called once
 *   with FMAC_XFER_ERROR_DMA, the Gpdma channels are stopped, the transfer state is FMAC_XFER_STATE_ERROR, the filter
 *   is FMAC_FILTER_STOPPED, the DMA request enables are cleared.
 * - A transfer complete callback afterwards does nothing; an error callback without a request is ignored.
 */
void Ut_Fmac_Set_FilterProcess_Dma_GpdmaError_ReportsAndAbortsRequest( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = UtFmac_Get_FirConfig();
    fmac_XferState_t    xferState    = FMAC_XFER_STATE_IDLE;
    fmac_XferError_t    xferError    = FMAC_XFER_ERROR_NONE;
    fmac_FilterState_t  filterState  = FMAC_FILTER_RUNNING;

    UtFmac_InitDma();
    UtFmac_Running( &filterConfig );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );

    UtFmac_ResetGpdmaLog();
    utFmac_DmaInitConfig[ 0u ].ErrorIsr( GPDMA_ERROR_TRANSFER );

    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_DMA, utFmac_LastErrorMask );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_GpdmaIdCnt[ UT_FMAC_GPDMA_CH_INACTIVE ] );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_ERROR, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferError( &xferError ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_DMA, xferError );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
    TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );

    utFmac_DmaInitConfig[ 1u ].TransferCompleteIsr();
    utFmac_DmaInitConfig[ 1u ].ErrorIsr( GPDMA_ERROR_CONFIG_ERROR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmac_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   The FMAC interrupt in DMA mode reports the error flags, the data are moved by Gpdma.
 *
 * \par Expected results
 * - No request in progress: the interrupt with the overflow flag set does nothing (no callback).
 * - A request in progress and FMAC_SR.SAT set: the error callback is called once with FMAC_XFER_ERROR_SATURATION, the
 *   request goes on (busy), the saturation interrupt is disabled.
 * - FMAC_SR.OVFL set: the error callback is called with FMAC_XFER_ERROR_OVERFLOW, the request ends with an error and
 *   the Gpdma channels are stopped.
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

    UtFmac_ResetGpdmaLog();
    FMAC->SR = FMAC_SR_SAT | FMAC_SR_OVFL;
    utFmac_Isr();
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_OVERFLOW, utFmac_LastErrorMask );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_TWICE, utFmac_GpdmaIdCnt[ UT_FMAC_GPDMA_CH_INACTIVE ] );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_ERROR, xferState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A step of the arming of the Gpdma channels which fails ends the request with an error.
 *
 * \par Expected results
 * - For each of the Gpdma calls of the arming (transfer list generation, link register, link, block size, source,
 *   destination, enable of the channel of the results and of the input data) failing in turn: Fmac_Set_FilterProcess()
 *   returns FMAC_REQUEST_ERROR, the transfer state is FMAC_XFER_STATE_ERROR, the filter is FMAC_FILTER_STOPPED and
 *   the DMA request enables are cleared.
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
        UtFmac_ResetGpdmaLog();
        utFmac_GpdmaFailId  = utFmac_ArmFailLut[ stepIdx ].Id;
        utFmac_GpdmaFailNth = utFmac_ArmFailLut[ stepIdx ].Nth;

        TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, UtFmac_StartBlock() );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
        TEST_ASSERT_EQUAL( FMAC_XFER_STATE_ERROR, xferState );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
        TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
        TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );

        utFmac_GpdmaFailId  = UT_FMAC_GPDMA_NONE;
        utFmac_GpdmaFailNth = 0u;
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterPreload( NULL, 0u, NULL, 0u ) );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );
    }

    UtFmac_ResetGpdmaLog();
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
#else
    TEST_IGNORE_MESSAGE( UT_FMAC_NO_PERIPH_MSG );
#endif /* FMAC */
}


/**
 * \brief   A dot product in DMA mode arms only the channel of the results.
 *
 * \par Expected results
 * - Fmac_Set_FilterProcess() with 4 inputs: the inputs are loaded into X1 by the CPU, the channel of the results is
 *   armed (block size 2 bytes), the channel of the input data is not, FMAC_CR has DMAREN and no DMAWEN.
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
    UtFmac_ResetGpdmaLog();
    utFmac_X1Cnt = 0u;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartDotProduct() );
    TEST_ASSERT_EQUAL_INT16_ARRAY( utFmac_Input, utFmac_X1Data, UT_FMAC_TAPS );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_ARM_CALLS, utFmac_GpdmaCnt );
    UtFmac_AssertArm( 0u,
                      GPDMA_CHANNEL_1,
                      UT_FMAC_BYTES_PER_SAMPLE,
                      UT_FMAC_ADDR( &FMAC->RDATA ),
                      UT_FMAC_ADDR( &utFmac_OutputMem[ UT_FMAC_GUARD_SIZE ] ) );
    TEST_ASSERT_BITS_HIGH( FMAC_CR_DMAREN, FMAC->CR );
    TEST_ASSERT_BITS_LOW( FMAC_CR_DMAWEN, FMAC->CR );

    utFmac_DmaInitConfig[ 1u ].TransferCompleteIsr();

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
 * \brief   A convolution in DMA mode arms both channels and enables both DMA requests.
 *
 * \par Expected results
 * - Fmac_Set_FilterProcess() with 6 samples: both channels are armed, FMAC_CR has DMAWEN and DMAREN, the CPU writes no
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
    UtFmac_ResetGpdmaLog();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, UtFmac_StartBlock() );
    TEST_ASSERT_EQUAL_UINT32( UT_FMAC_DMA_STREAMS * UT_FMAC_ARM_CALLS, utFmac_GpdmaCnt );
    TEST_ASSERT_BITS_HIGH( FMAC_CR_DMAWEN | FMAC_CR_DMAREN, FMAC->CR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmac_RunCnt );

    utFmac_DmaInitConfig[ 1u ].TransferCompleteIsr();

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
        if( ( UT_FMAC_GUARD_SIZE > sampleIdx ) ||
            ( ( UT_FMAC_GUARD_SIZE + resultCnt ) <= sampleIdx ) )
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
 * \brief Records a call of a Gpdma function and returns its result: the call number \ref utFmac_GpdmaFailNth of the
 *        function \ref utFmac_GpdmaFailId fails.
 *
 * \param callId    [in]: Gpdma function.
 * \param periphId  [in]: Gpdma controller of the call (\ref UT_FMAC_NA when the function has none).
 * \param channelId [in]: Gpdma channel of the call (\ref UT_FMAC_NA when the function has none).
 * \param value     [in]: Value the test checks (block size, address, link register, channel type, ...).
 *
 * \return Result of the Gpdma function.
 */
static gpdma_RequestState_t UtFmac_GpdmaRecord( utFmac_GpdmaCall_t callId,
                                                uint32_t periphId,
                                                uint32_t channelId,
                                                uint32_t value )
{
    gpdma_RequestState_t retState = GPDMA_REQUEST_OK;

    if( UT_FMAC_DMA_LOG_SIZE > utFmac_GpdmaCnt )
    {
        utFmac_GpdmaLog[ utFmac_GpdmaCnt ].Id      = callId;
        utFmac_GpdmaLog[ utFmac_GpdmaCnt ].Periph  = periphId;
        utFmac_GpdmaLog[ utFmac_GpdmaCnt ].Channel = channelId;
        utFmac_GpdmaLog[ utFmac_GpdmaCnt ].Value   = value;
    }
    else
    {
        /* The log is full */
    }

    utFmac_GpdmaCnt ++;
    utFmac_GpdmaIdCnt[ callId ] ++;

    if( ( utFmac_GpdmaFailId  == callId                         ) &&
        ( utFmac_GpdmaFailNth == utFmac_GpdmaIdCnt[ callId ]    )    )
    {
        retState = GPDMA_REQUEST_ERROR;
    }
    else
    {
        /* The call succeeds */
    }

    return ( retState );
}


/** Stub of Gpdma_Init: keeps a copy of the configuration (and of its transfer) of the first two calls */
static gpdma_RequestState_t UtFmac_GpdmaInitStub( gpdma_ConfigStruct_t * const configStruct, int callCnt )
{
    const uint32_t slot = utFmac_GpdmaIdCnt[ UT_FMAC_GPDMA_INIT ];

    (void)callCnt;

    if( UT_FMAC_DMA_STREAMS > slot )
    {
        utFmac_DmaInitConfig[ slot ]                = *configStruct;
        utFmac_DmaInitXfer[ slot ]                  = *configStruct->TransferConfig;
        utFmac_DmaInitConfig[ slot ].TransferConfig = &utFmac_DmaInitXfer[ slot ];
    }
    else
    {
        /* Only the two channels of the module are kept */
    }

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_INIT, configStruct->PeriphId, configStruct->ChannelId, 0u ) );
}


/** Stub of Gpdma_Deinit */
static gpdma_RequestState_t UtFmac_GpdmaDeinitStub( gpdma_PeriphId_t periphId,
                                                    gpdma_ChannelId_t channelId,
                                                    int callCnt )
{
    (void)callCnt;

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_DEINIT, periphId, channelId, 0u ) );
}


/** Stub of Gpdma_Set_InterruptActive */
static gpdma_RequestState_t UtFmac_GpdmaIrqActiveStub( gpdma_PeriphId_t periphId,
                                                       gpdma_ChannelId_t channelId,
                                                       int callCnt )
{
    (void)callCnt;

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_IRQ_ACTIVE, periphId, channelId, 0u ) );
}


/** Stub of Gpdma_Get_XferListConfig: keeps a copy of the transfer configuration of the first two calls */
static gpdma_RequestState_t UtFmac_GpdmaListConfigStub( gpdma_TransferConfig_t * const transferConfig,
                                                        gpdma_TransfersCount_t transferCount,
                                                        gpdma_ChannelType_t channelType,
                                                        volatile gpdma_XferList_t * transferList,
                                                        int callCnt )
{
    const uint32_t slot = utFmac_GpdmaIdCnt[ UT_FMAC_GPDMA_LIST_CONFIG ];

    (void)transferCount;
    (void)transferList;
    (void)callCnt;

    if( UT_FMAC_DMA_STREAMS > slot )
    {
        utFmac_DmaXfer[ slot ] = *transferConfig;
    }
    else
    {
        /* Only the two transfers of a request are kept */
    }

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_LIST_CONFIG, UT_FMAC_NA, UT_FMAC_NA, (uint32_t)channelType ) );
}


/** Stub of Gpdma_Get_XferList_LinkReg: returns the marker as the link register */
static gpdma_RequestState_t UtFmac_GpdmaLinkRegStub( volatile gpdma_XferList_t * const transferList,
                                                     gpdma_ChannelType_t channelType,
                                                     gpdma_XferLinkReg_t * const linkReg,
                                                     int callCnt )
{
    (void)transferList;
    (void)callCnt;

    *linkReg = UT_FMAC_LINKREG_MARK;

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_LINK_REG, UT_FMAC_NA, UT_FMAC_NA, (uint32_t)channelType ) );
}


/** Stub of Gpdma_Set_XferListLink */
static gpdma_RequestState_t UtFmac_GpdmaListLinkStub( gpdma_PeriphId_t periphId,
                                                      gpdma_ChannelId_t channelId,
                                                      gpdma_XferLinkReg_t linkReg,
                                                      int callCnt )
{
    (void)callCnt;

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_LIST_LINK, periphId, channelId, linkReg ) );
}


/** Stub of Gpdma_Set_BlockSize */
static gpdma_RequestState_t UtFmac_GpdmaBlockSizeStub( gpdma_PeriphId_t periphId,
                                                       gpdma_ChannelId_t channelId,
                                                       gpdma_BlockSize_t blockSize,
                                                       int callCnt )
{
    (void)callCnt;

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_BLOCK_SIZE, periphId, channelId, blockSize ) );
}


/** Stub of Gpdma_Set_SourceAddr */
static gpdma_RequestState_t UtFmac_GpdmaSrcAddrStub( gpdma_PeriphId_t periphId,
                                                     gpdma_ChannelId_t channelId,
                                                     gpdma_SrcAddr_t sourceAddr,
                                                     int callCnt )
{
    (void)callCnt;

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_SRC_ADDR, periphId, channelId, sourceAddr ) );
}


/** Stub of Gpdma_Set_DestinationAddr */
static gpdma_RequestState_t UtFmac_GpdmaDstAddrStub( gpdma_PeriphId_t periphId,
                                                     gpdma_ChannelId_t channelId,
                                                     gpdma_DstAddr_t destAddr,
                                                     int callCnt )
{
    (void)callCnt;

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_DST_ADDR, periphId, channelId, destAddr ) );
}


/** Stub of Gpdma_Set_ChannelActive */
static gpdma_RequestState_t UtFmac_GpdmaChActiveStub( gpdma_PeriphId_t periphId,
                                                      gpdma_ChannelId_t channelId,
                                                      int callCnt )
{
    (void)callCnt;

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_CH_ACTIVE, periphId, channelId, 0u ) );
}


/** Stub of Gpdma_Set_ChannelInactive */
static gpdma_RequestState_t UtFmac_GpdmaChInactiveStub( gpdma_PeriphId_t periphId,
                                                        gpdma_ChannelId_t channelId,
                                                        int callCnt )
{
    (void)callCnt;

    return ( UtFmac_GpdmaRecord( UT_FMAC_GPDMA_CH_INACTIVE, periphId, channelId, 0u ) );
}


/** Installs the stubs of the Gpdma functions the module calls */
static void UtFmac_GpdmaStubs( void )
{
    Gpdma_Init_Stub( UtFmac_GpdmaInitStub );
    Gpdma_Deinit_Stub( UtFmac_GpdmaDeinitStub );
    Gpdma_Set_InterruptActive_Stub( UtFmac_GpdmaIrqActiveStub );
    Gpdma_Get_XferListConfig_Stub( UtFmac_GpdmaListConfigStub );
    Gpdma_Get_XferList_LinkReg_Stub( UtFmac_GpdmaLinkRegStub );
    Gpdma_Set_XferListLink_Stub( UtFmac_GpdmaListLinkStub );
    Gpdma_Set_BlockSize_Stub( UtFmac_GpdmaBlockSizeStub );
    Gpdma_Set_SourceAddr_Stub( UtFmac_GpdmaSrcAddrStub );
    Gpdma_Set_DestinationAddr_Stub( UtFmac_GpdmaDstAddrStub );
    Gpdma_Set_ChannelActive_Stub( UtFmac_GpdmaChActiveStub );
    Gpdma_Set_ChannelInactive_Stub( UtFmac_GpdmaChInactiveStub );
}


/** Clears the log of the Gpdma calls and the counters of the calls (the copies of the Init configurations stay) */
static void UtFmac_ResetGpdmaLog( void )
{
    utFmac_GpdmaCnt = 0u;

    for( uint32_t callIdx = 0u; UT_FMAC_GPDMA_CNT > callIdx; callIdx ++ )
    {
        utFmac_GpdmaIdCnt[ callIdx ] = 0u;
    }
}


/**
 * \brief Checks one record of the log of the Gpdma calls.
 *
 * \param logIdx    [in]: Index into the log.
 * \param callId    [in]: Expected Gpdma function.
 * \param periphId  [in]: Expected controller.
 * \param channelId [in]: Expected channel.
 * \param value     [in]: Expected value of the call.
 */
static void UtFmac_AssertGpdma( uint32_t logIdx,
                                utFmac_GpdmaCall_t callId,
                                uint32_t periphId,
                                uint32_t channelId,
                                uint32_t value )
{
    TEST_ASSERT_TRUE( UT_FMAC_DMA_LOG_SIZE > logIdx );
    TEST_ASSERT_EQUAL_UINT32( callId, utFmac_GpdmaLog[ logIdx ].Id );
    TEST_ASSERT_EQUAL_UINT32( periphId, utFmac_GpdmaLog[ logIdx ].Periph );
    TEST_ASSERT_EQUAL_UINT32( channelId, utFmac_GpdmaLog[ logIdx ].Channel );
    TEST_ASSERT_EQUAL_HEX32( value, utFmac_GpdmaLog[ logIdx ].Value );
}


/**
 * \brief Checks the seven Gpdma calls which arm a channel for a request.
 *
 * \param firstIdx  [in]: Index of the first call in the log.
 * \param channelId [in]: Channel of the stream.
 * \param blockSize [in]: Expected block size in bytes.
 * \param srcAddr   [in]: Expected source address.
 * \param dstAddr   [in]: Expected destination address.
 */
static void UtFmac_AssertArm( uint32_t firstIdx,
                              gpdma_ChannelId_t channelId,
                              uint32_t blockSize,
                              uint32_t srcAddr,
                              uint32_t dstAddr )
{
    uint32_t logIdx = firstIdx;

    UtFmac_AssertGpdma( logIdx ++, UT_FMAC_GPDMA_LIST_CONFIG, UT_FMAC_NA, UT_FMAC_NA, GPDMA_CHANNEL_LINEAR );
    UtFmac_AssertGpdma( logIdx ++, UT_FMAC_GPDMA_LINK_REG, UT_FMAC_NA, UT_FMAC_NA, GPDMA_CHANNEL_LINEAR );
    UtFmac_AssertGpdma( logIdx ++, UT_FMAC_GPDMA_LIST_LINK, GPDMA_PERIPH_1, channelId, UT_FMAC_LINKREG_MARK );
    UtFmac_AssertGpdma( logIdx ++, UT_FMAC_GPDMA_BLOCK_SIZE, GPDMA_PERIPH_1, channelId, blockSize );
    UtFmac_AssertGpdma( logIdx ++, UT_FMAC_GPDMA_SRC_ADDR, GPDMA_PERIPH_1, channelId, srcAddr );
    UtFmac_AssertGpdma( logIdx ++, UT_FMAC_GPDMA_DST_ADDR, GPDMA_PERIPH_1, channelId, dstAddr );
    UtFmac_AssertGpdma( logIdx, UT_FMAC_GPDMA_CH_ACTIVE, GPDMA_PERIPH_1, channelId, 0u );
}


/**
 * \brief Checks the transfer configuration the module generated for a stream of the request (16 bit data, one
 *        request per sample, fixed peripheral address, incrementing memory address, no trigger).
 *
 * \param slot      [in]: 0 for the first transfer of the request (results), 1 for the second (input data).
 * \param direction [in]: Expected direction.
 * \param request   [in]: Expected Gpdma request of the FMAC.
 * \param execMode  [in]: Expected execution mode of the transfer list.
 */
static void UtFmac_AssertXfer( uint32_t slot,
                               gpdma_Direction_t direction,
                               gpdma_PeriphReqId_t request,
                               gpdma_XferListExecMode_t execMode )
{
    const gpdma_TransferConfig_t * const xfer = &utFmac_DmaXfer[ slot ];

    TEST_ASSERT_EQUAL( direction, xfer->Direction );
    TEST_ASSERT_EQUAL( request, xfer->RequestSource );
    TEST_ASSERT_EQUAL( execMode, xfer->XferListExecMode );
    TEST_ASSERT_EQUAL( GPDMA_PERIPH_REQ_SINGLE, xfer->RequestMode );
    TEST_ASSERT_EQUAL( GPDMA_TRG_NOT_USED, xfer->TriggerType );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_16BITS, xfer->SourceDataSize );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_16BITS, xfer->DestinationDataSize );

    if( GPDMA_DIR_MEMORY_TO_PERIPH == direction )
    {
        TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT, xfer->SourceAddrMode );
        TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC, xfer->DestinationAddrMode );
    }
    else
    {
        TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC, xfer->SourceAddrMode );
        TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT, xfer->DestinationAddrMode );
    }
}


/** Activates the HW model and initializes the module in DMA mode, the log of the Gpdma calls is cleared */
static void UtFmac_InitDma( void )
{
    UtFmac_Init( FMAC_XFER_MODE_DMA );
    UtFmac_ResetGpdmaLog();
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
