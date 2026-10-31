/**
 * \author Mr.Nobody
 * \file ItTest_Fmac.c
 * \ingroup Fmac
 * \brief Integration tests of the Fmac module on target.
 *
 * The Fmac module runs on the MCU together with the real FMAC / GPDMA / RCC / NVIC hardware. The accelerator needs no
 * wiring and no fixture: every test runs on a bare board. The tests filter short sequences of samples with the FIR
 * filter, the IIR filter and the vector functions (dot product, convolution, correlation) in the three transfer modes
 * (POLL, ISR, DMA with two Gpdma channels) and compare the results with a reference calculation of the same q1.15
 * arithmetic (the coefficients and the samples are chosen so that no result is rounded: the accumulator is a sum of
 * products, shifted right by 15 bits). One shot and circular streams are started and stopped, the filter state, the
 * transfer state and the error flags are checked.
 *
 * Used resources:
 * - FMAC (kernel clock = bus clock)
 * - GPDMA1 channel 0 (input samples) and channel 1 (results) in the DMA tests
 *
 * \note On the lines without the FMAC (STM32H503, H523, H533, H543, H553) every test is ignored
 *       (`TEST_IGNORE_MESSAGE`), see the module README availability table.
 * \note The buffers of the DMA tests are in the RAM of the program, the lines of the STM32H5 have no data cache that
 *       would need a clean or an invalidate around the transfers of the Gpdma.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Fmac_Port.h"                      /* Module under test              */
#include "stdint.h"                         /* Standard integer types         */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(FMAC)
static void                ItFmac_XferHalfCallback    ( void );
static void                ItFmac_XferCompleteCallback( void );
static void                ItFmac_ErrorCallback       ( fmac_XferError_t errorMask );
static fmac_FilterConfig_t ItFmac_Get_FirConfig       ( void );
static fmac_FilterConfig_t ItFmac_Get_IirConfig       ( void );
static fmac_FilterConfig_t ItFmac_Get_VectorConfig    ( fmac_Function_t function );
static fmac_RequestState_t ItFmac_Start               ( fmac_XferMode_t xferMode,
                                                        const fmac_FilterConfig_t * const filterConfig );
static fmac_RequestState_t ItFmac_Process             ( const fmac_Sample_t * const input );
static void                ItFmac_Wait_Idle           ( void );
static void                ItFmac_Wait_Laps           ( uint32_t lapCnt );
static void                ItFmac_Ref_Fir             ( const fmac_Sample_t * const coef,
                                                        fmac_SampleCnt_t taps,
                                                        const fmac_Sample_t * const input,
                                                        fmac_SampleCnt_t sampleCnt );
static void                ItFmac_Ref_Iir             ( void );
static void                ItFmac_Ref_Correlation     ( void );
static void                ItFmac_Ref_Dot             ( void );
static void                ItFmac_Assert_Result       ( fmac_SampleCnt_t resultCnt );
static void                ItFmac_Run_Block           ( fmac_XferMode_t xferMode );
static void                ItFmac_Run_Vector          ( fmac_XferMode_t xferMode, fmac_Function_t function );
#endif /* FMAC */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Message of a test on a line without the FMAC */
#define IT_FMAC_NO_FMAC_MSG                 "No FMAC on this line (STM32H503 / H523 / H533 / H543 / H553)."

/** Interrupt priority used by the tests */
#define IT_FMAC_IRQ_PRIO                    ( 5u )

/** Count of the samples of a request and of the half of them */
#define IT_FMAC_SAMPLE_CNT                  ( 8u )

/** Count of the coefficients of the FIR filter and of the vector functions */
#define IT_FMAC_TAPS                        ( 4u )

/** Count of the feedforward and of the feedback coefficients of the IIR filter */
#define IT_FMAC_IIR_TAPS                    ( 2u )
#define IT_FMAC_IIR_FEEDBACK                ( 1u )

/** Count of the blocks of the test with two consecutive blocks */
#define IT_FMAC_BLOCK_CNT                   ( 2u )

/** Right shift of the accumulator that gives a q1.15 result (gain 1) */
#define IT_FMAC_Q15_SHIFT                   ( 15u )

/** Limit of the loops waiting for the end of a request or for the laps of a stream */
#define IT_FMAC_WAIT_LIMIT                  ( 2000000u )

/** Count of the laps of a circular stream the test waits for */
#define IT_FMAC_STREAM_LAPS                 ( 3u )

/** Size of the buffers of the circular stream in the DMA mode (samples): a lap of 8 samples takes about as long as
 *  its interrupt of the Gpdma, the CPU would be busy with the interrupts and the test would never continue */
#define IT_FMAC_STREAM_SAMPLE_CNT           ( 512u )

/** Memory usage in words of the FIR filter of the tests: 4 coefficients, 5 input and 1 output samples */
#define IT_FMAC_FIR_WORDS                   ( 10u )

/** Conversion test values and the samples they give */
#define IT_FMAC_FLOAT_HALF                  ( 0.5f )
#define IT_FMAC_FLOAT_TOO_LARGE             ( 2.0f )
#define IT_FMAC_SAMPLE_HALF                 ( (fmac_Sample_t)16384 )
#define IT_FMAC_SAMPLE_MAX                  ( (fmac_Sample_t)32767 )

/* ========================== LOCAL VARIABLES =============================== */

#if defined(FMAC)

/** Coefficients of the FIR filter and of the vector functions (0.25 each, then 0.125 .. 0.5 in steps of 0.125) */
static const fmac_Sample_t itFmac_FirCoef[ IT_FMAC_TAPS ] = { 8192, 8192, 8192, 8192 };
static const fmac_Sample_t itFmac_VectorCoef[ IT_FMAC_TAPS ] = { 4096, 8192, 12288, 16384 };

/** Coefficients of the IIR filter: y[n] = 0.5 * x[n] + 0.5 * y[n-1] */
static const fmac_Sample_t itFmac_IirB[ IT_FMAC_IIR_TAPS ] = { 16384, 0 };
static const fmac_Sample_t itFmac_IirA[ IT_FMAC_IIR_FEEDBACK ] = { 16384 };

/** Input samples of the FIR filter and of the vector functions (multiples of 8: no rounding of the results) */
static const fmac_Sample_t itFmac_Input[ IT_FMAC_SAMPLE_CNT ] = { 400, 800, 1200, 1600, -2000, -2400, 2800, 3200 };

/** Input samples of the IIR filter (a step: the results are 1024, 1536, 1792 ...) */
static const fmac_Sample_t itFmac_IirInput[ IT_FMAC_SAMPLE_CNT ] = { 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048 };

/** Results of the accelerator and the expected results of the reference calculation (two blocks for the test of the
 *  state kept between two requests) */
static fmac_Sample_t itFmac_Output[ IT_FMAC_SAMPLE_CNT ];
static fmac_Sample_t itFmac_Expected[ IT_FMAC_BLOCK_CNT * IT_FMAC_SAMPLE_CNT ];

/** Buffers of the circular stream in the DMA mode: the long laps leave CPU time between the Gpdma interrupts */
static fmac_Sample_t itFmac_StreamInput[ IT_FMAC_STREAM_SAMPLE_CNT ];
static fmac_Sample_t itFmac_StreamOutput[ IT_FMAC_STREAM_SAMPLE_CNT ];

/** Counters of the callbacks and the errors of the last callback */
static volatile uint32_t         itFmac_HalfCnt;
static volatile uint32_t         itFmac_CompleteCnt;
static volatile uint32_t         itFmac_ErrorCnt;
static volatile fmac_XferError_t itFmac_LastError;

#endif /* FMAC */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    /* No per-test state */
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* =========================== LOCAL FUNCTIONS ============================== */

#if defined(FMAC)

/** Half of the output buffer callback of the tests: counts the calls */
static void ItFmac_XferHalfCallback( void )
{
    itFmac_HalfCnt ++;
}


/** End of the output buffer callback of the tests: counts the calls */
static void ItFmac_XferCompleteCallback( void )
{
    itFmac_CompleteCnt ++;
}


/** Error callback of the tests: counts the calls and keeps the last error mask */
static void ItFmac_ErrorCallback( fmac_XferError_t errorMask )
{
    itFmac_ErrorCnt ++;
    itFmac_LastError = errorMask;
}


/** Returns the configuration of the FIR filter of the tests: 4 coefficients of 0.25, gain 1, saturation */
static fmac_FilterConfig_t ItFmac_Get_FirConfig( void )
{
    fmac_FilterConfig_t filterConfig = { 0 };

    filterConfig.Function = FMAC_FUNC_FIR;
    filterConfig.B        = itFmac_FirCoef;
    filterConfig.BCount   = IT_FMAC_TAPS;
    filterConfig.Gain     = FMAC_GAIN_1;
    filterConfig.Clipping = FMAC_CLIP_SATURATE;

    return ( filterConfig );
}


/** Returns the configuration of the IIR filter of the tests: y[n] = 0.5 * x[n] + 0.5 * y[n-1] */
static fmac_FilterConfig_t ItFmac_Get_IirConfig( void )
{
    fmac_FilterConfig_t filterConfig = { 0 };

    filterConfig.Function = FMAC_FUNC_IIR;
    filterConfig.B        = itFmac_IirB;
    filterConfig.BCount   = IT_FMAC_IIR_TAPS;
    filterConfig.A        = itFmac_IirA;
    filterConfig.ACount   = IT_FMAC_IIR_FEEDBACK;
    filterConfig.Gain     = FMAC_GAIN_1;
    filterConfig.Clipping = FMAC_CLIP_SATURATE;

    return ( filterConfig );
}


/**
 * \brief Returns the configuration of a vector function with the coefficients of the tests.
 *
 * \param function [in]: Dot product, convolution or correlation.
 *
 * \return Filter configuration.
 */
static fmac_FilterConfig_t ItFmac_Get_VectorConfig( fmac_Function_t function )
{
    fmac_FilterConfig_t filterConfig = ItFmac_Get_FirConfig();

    filterConfig.Function = function;
    filterConfig.B        = itFmac_VectorCoef;

    return ( filterConfig );
}


/**
 * \brief Initializes the module in a transfer mode, configures the filter and starts it.
 *
 * \param xferMode     [in]: Transfer mode.
 * \param filterConfig [in]: Filter configuration.
 *
 * \return FMAC_REQUEST_OK when every step succeeded. Otherwise FMAC_REQUEST_ERROR.
 */
static fmac_RequestState_t ItFmac_Start( fmac_XferMode_t xferMode, const fmac_FilterConfig_t * const filterConfig )
{
    fmac_Config_t       config   = { 0 };
    fmac_RequestState_t retState = Fmac_Get_DefaultConfig( &config );

    itFmac_HalfCnt     = 0u;
    itFmac_CompleteCnt = 0u;
    itFmac_ErrorCnt    = 0u;
    itFmac_LastError   = FMAC_XFER_ERROR_NONE;

    for( uint32_t sampleIdx = 0u; ( IT_FMAC_SAMPLE_CNT > sampleIdx ); sampleIdx++ )
    {
        itFmac_Output[ sampleIdx ] = 0;
    }

    config.XferMode             = xferMode;
    config.IrqPriority          = IT_FMAC_IRQ_PRIO;
    config.XferHalfCallback     = ItFmac_XferHalfCallback;
    config.XferCompleteCallback = ItFmac_XferCompleteCallback;
    config.ErrorCallback        = ItFmac_ErrorCallback;

    if( FMAC_REQUEST_OK == retState )
    {
        retState = Fmac_Init( &config );
    }
    else
    {
        /* The default configuration is not available */
    }

    if( FMAC_REQUEST_OK == retState )
    {
        retState = Fmac_Set_FilterConfig( filterConfig );
    }
    else
    {
        /* The module is not initialized */
    }

    if( FMAC_REQUEST_OK == retState )
    {
        retState = Fmac_Set_FilterStart();
    }
    else
    {
        /* The filter is not configured */
    }

    return ( retState );
}


/**
 * \brief Starts a block request with 8 input samples and the result buffer of the tests.
 *
 * \param input [in]: Input samples.
 *
 * \return Result of Fmac_Set_FilterProcess().
 */
static fmac_RequestState_t ItFmac_Process( const fmac_Sample_t * const input )
{
    return ( Fmac_Set_FilterProcess( input, IT_FMAC_SAMPLE_CNT, itFmac_Output, IT_FMAC_SAMPLE_CNT ) );
}


/** Waits (bounded) until the request in progress ends: Fmac_Task() serves it in POLL mode, the interrupt or the Gpdma
 *  in the other modes */
static void ItFmac_Wait_Idle( void )
{
    fmac_XferState_t xferState = FMAC_XFER_STATE_BUSY;

    for( uint32_t waitIdx = 0u; IT_FMAC_WAIT_LIMIT > waitIdx; waitIdx++ )
    {
        if( FMAC_XFER_STATE_BUSY == xferState )
        {
            Fmac_Task();
            ( void )Fmac_Get_XferState( &xferState );
        }
        else
        {
            /* The request ended */
            break;
        }
    }
}


/**
 * \brief Waits (bounded) until a stream completed a count of laps.
 *
 * \param lapCnt [in]: Count of the laps (complete callbacks) to wait for.
 */
static void ItFmac_Wait_Laps( uint32_t lapCnt )
{
    for( uint32_t waitIdx = 0u; IT_FMAC_WAIT_LIMIT > waitIdx; waitIdx++ )
    {
        if( lapCnt > itFmac_CompleteCnt )
        {
            Fmac_Task();
        }
        else
        {
            /* The laps are done */
            break;
        }
    }
}


/**
 * \brief Reference calculation of a FIR filter or of a convolution with the zero state: y[n] = sum( B[k] * x[n-k] )
 *        shifted right by 15 bits, into \ref itFmac_Expected.
 *
 * \param coef      [in]: Coefficients.
 * \param taps      [in]: Count of the coefficients.
 * \param input     [in]: Input samples.
 * \param sampleCnt [in]: Count of the input samples.
 */
static void ItFmac_Ref_Fir( const fmac_Sample_t * const coef,
                            fmac_SampleCnt_t taps,
                            const fmac_Sample_t * const input,
                            fmac_SampleCnt_t sampleCnt )
{
    for( fmac_SampleCnt_t sampleIdx = 0u; ( sampleCnt > sampleIdx ); sampleIdx++ )
    {
        int32_t accumulator = 0;

        for( fmac_SampleCnt_t tapIdx = 0u; taps > tapIdx; tapIdx++ )
        {
            if( sampleIdx >= tapIdx )
            {
                accumulator += ( (int32_t)coef[ tapIdx ] * (int32_t)input[ sampleIdx - tapIdx ] );
            }
            else
            {
                /* The sample before the first one is zero */
            }
        }

        itFmac_Expected[ sampleIdx ] = (fmac_Sample_t)( accumulator >> IT_FMAC_Q15_SHIFT );
    }
}


/** Reference calculation of the IIR filter of the tests with a zero state into \ref itFmac_Expected */
static void ItFmac_Ref_Iir( void )
{
    for( fmac_SampleCnt_t sampleIdx = 0u; ( IT_FMAC_SAMPLE_CNT > sampleIdx ); sampleIdx++ )
    {
        int32_t accumulator = ( (int32_t)itFmac_IirB[ 0 ] * (int32_t)itFmac_IirInput[ sampleIdx ] );

        if( 0u < sampleIdx )
        {
            accumulator += ( (int32_t)itFmac_IirB[ 1 ] * (int32_t)itFmac_IirInput[ sampleIdx - 1u ] );
            accumulator += ( (int32_t)itFmac_IirA[ 0 ] * (int32_t)itFmac_Expected[ sampleIdx - 1u ] );
        }
        else
        {
            /* The state is zero before the first sample */
        }

        itFmac_Expected[ sampleIdx ] = (fmac_Sample_t)( accumulator >> IT_FMAC_Q15_SHIFT );
    }
}


/** Reference calculation of the correlation of the tests with a zero state: y[n] = sum( B[k] * x[n - N + 1 + k] ) */
static void ItFmac_Ref_Correlation( void )
{
    for( fmac_SampleCnt_t sampleIdx = 0u; ( IT_FMAC_SAMPLE_CNT > sampleIdx ); sampleIdx++ )
    {
        int32_t accumulator = 0;

        for( fmac_SampleCnt_t tapIdx = 0u; ( IT_FMAC_TAPS > tapIdx ); tapIdx++ )
        {
            const fmac_SampleCnt_t inputIdx = ( ( sampleIdx + 1u + tapIdx ) - IT_FMAC_TAPS );

            if( ( IT_FMAC_TAPS <= ( sampleIdx + 1u + tapIdx ) ) &&
                ( IT_FMAC_SAMPLE_CNT > inputIdx             )    )
            {
                accumulator += ( (int32_t)itFmac_VectorCoef[ tapIdx ] * (int32_t)itFmac_Input[ inputIdx ] );
            }
            else
            {
                /* The sample before the first one is zero */
            }
        }

        itFmac_Expected[ sampleIdx ] = (fmac_Sample_t)( accumulator >> IT_FMAC_Q15_SHIFT );
    }
}


/** Reference calculation of the dot product of the first 4 test inputs and the vector coefficients */
static void ItFmac_Ref_Dot( void )
{
    int32_t accumulator = 0;

    for( fmac_SampleCnt_t tapIdx = 0u; ( IT_FMAC_TAPS > tapIdx ); tapIdx++ )
    {
        accumulator += ( (int32_t)itFmac_VectorCoef[ tapIdx ] * (int32_t)itFmac_Input[ tapIdx ] );
    }

    itFmac_Expected[ 0 ] = (fmac_Sample_t)( accumulator >> IT_FMAC_Q15_SHIFT );
}


/**
 * \brief Checks the results of the last request against the reference calculation and that no error was reported.
 *
 * \param resultCnt [in]: Count of the results.
 */
static void ItFmac_Assert_Result( fmac_SampleCnt_t resultCnt )
{
    fmac_XferError_t xferError = FMAC_XFER_ERROR_ALL;

    TEST_ASSERT_EQUAL_INT16_ARRAY( itFmac_Expected, itFmac_Output, resultCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferError( &xferError ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_ERROR_NONE, xferError );
}


/**
 * \brief Filters a block with the FIR filter in a transfer mode and compares the results with the reference.
 *
 * \param xferMode [in]: Transfer mode.
 */
static void ItFmac_Run_Block( fmac_XferMode_t xferMode )
{
    const fmac_FilterConfig_t filterConfig = ItFmac_Get_FirConfig();
    fmac_XferState_t          xferState    = FMAC_XFER_STATE_BUSY;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Start( xferMode, &filterConfig ) );
    ItFmac_Ref_Fir( itFmac_FirCoef, IT_FMAC_TAPS, itFmac_Input, IT_FMAC_SAMPLE_CNT );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Process( itFmac_Input ) );
    ItFmac_Wait_Idle();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL_UINT32( 1u, itFmac_CompleteCnt );
    ItFmac_Assert_Result( IT_FMAC_SAMPLE_CNT );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
}


/**
 * \brief Runs a vector function on the test inputs in a transfer mode and compares the results with the reference.
 *
 * \param xferMode [in]: Transfer mode.
 * \param function [in]: Dot product, convolution or correlation.
 */
static void ItFmac_Run_Vector( fmac_XferMode_t xferMode, fmac_Function_t function )
{
    const fmac_FilterConfig_t filterConfig = ItFmac_Get_VectorConfig( function );
    fmac_SampleCnt_t          inputCnt     = IT_FMAC_SAMPLE_CNT;
    fmac_SampleCnt_t          resultCnt    = IT_FMAC_SAMPLE_CNT;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Start( xferMode, &filterConfig ) );

    if( FMAC_FUNC_DOT_PRODUCT == function )
    {
        inputCnt  = IT_FMAC_TAPS;
        resultCnt = 1u;
        ItFmac_Ref_Dot();
    }
    else if( FMAC_FUNC_CONVOLUTION == function )
    {
        ItFmac_Ref_Fir( itFmac_VectorCoef, IT_FMAC_TAPS, itFmac_Input, IT_FMAC_SAMPLE_CNT );
    }
    else
    {
        ItFmac_Ref_Correlation();
    }

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterProcess( itFmac_Input, inputCnt, itFmac_Output, resultCnt ) );
    ItFmac_Wait_Idle();

    ItFmac_Assert_Result( resultCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
}

#endif /* FMAC */

/* =============================== TESTS ==================================== */

/**
 * \brief The module version is reported without a fault.
 * \par Expected results
 * \ref Fmac_Get_ModuleVersion returns the major version 0.
 */
void It_Fmac_Get_ModuleVersion_ReturnsVersion( void )
{
    fmac_ModuleVersion_t version = Fmac_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 0u, version.Major );
}


/**
 * \brief The module initializes and deinitializes in the three transfer modes.
 * \par Expected results
 * \ref Fmac_Init returns \ref FMAC_REQUEST_OK in the POLL, ISR and DMA mode, a second init returns
 * \ref FMAC_REQUEST_ERROR, \ref Fmac_Deinit returns \ref FMAC_REQUEST_OK, a second deinit returns
 * \ref FMAC_REQUEST_ERROR.
 */
void It_Fmac_Init_AllModes_InitDeinitOk( void )
{
#if defined(FMAC)
    fmac_Config_t config;

    for( uint32_t modeIdx = 0u; ( (uint32_t)FMAC_XFER_MODE_CNT > modeIdx ); modeIdx++ )
    {
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_DefaultConfig( &config ) );
        config.XferMode    = (fmac_XferMode_t)modeIdx;
        config.IrqPriority = IT_FMAC_IRQ_PRIO;

        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Init( &config ) );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Init( &config ) );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
        TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Deinit() );
    }
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The memory usage of a filter is reported, the filter is configured, started and stopped.
 * \par Expected results
 * \ref Fmac_Get_MemoryUsage returns 10 words for the FIR filter of the tests, the filter state follows the calls
 * (ready, running, stopped), a request before the start and a start after the stop return \ref FMAC_REQUEST_ERROR.
 */
void It_Fmac_Set_FilterConfig_Fir_StateFollowsCalls( void )
{
#if defined(FMAC)
    const fmac_FilterConfig_t filterConfig = ItFmac_Get_FirConfig();
    fmac_Config_t             config;
    fmac_SampleCnt_t          wordCnt      = 0u;
    fmac_FilterState_t        filterState  = FMAC_FILTER_UNCONFIGURED;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_MemoryUsage( &filterConfig, &wordCnt ) );
    TEST_ASSERT_EQUAL_UINT32( IT_FMAC_FIR_WORDS, wordCnt );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Init( &config ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_UNCONFIGURED, filterState );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterConfig( &filterConfig ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_READY, filterState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, ItFmac_Process( itFmac_Input ) );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStart() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_RUNNING, filterState );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_FilterStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterStart() );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief A filter which does not fit the local memory of the accelerator is rejected.
 * \par Expected results
 * \ref Fmac_Set_FilterConfig returns \ref FMAC_REQUEST_ERROR for a FIR filter with the largest count of the
 * coefficients and a headroom of the input buffer (more than 256 words), the filter stays unconfigured.
 */
void It_Fmac_Set_FilterConfig_TooLarge_ReturnsError( void )
{
#if defined(FMAC)
    fmac_FilterConfig_t filterConfig = ItFmac_Get_FirConfig();
    fmac_Config_t       config;
    fmac_FilterState_t  filterState  = FMAC_FILTER_READY;
    fmac_SampleCnt_t    wordCnt      = 0u;

    filterConfig.BCount        = FMAC_FIR_COEF_MAX;
    filterConfig.InputHeadroom = FMAC_FIR_COEF_MAX;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_MemoryUsage( &filterConfig, &wordCnt ) );
    TEST_ASSERT_TRUE( FMAC_MEMORY_WORDS < wordCnt );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Init( &config ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_FilterConfig( &filterConfig ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_UNCONFIGURED, filterState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The FIR filter filters a block in the POLL mode (Fmac_Task moves the samples).
 * \par Expected results
 * The 8 results are those of the reference calculation, the complete callback is called once, no error is reported,
 * the transfer state returns to idle.
 */
void It_Fmac_Set_FilterProcess_FirPoll_ResultsMatchReference( void )
{
#if defined(FMAC)
    ItFmac_Run_Block( FMAC_XFER_MODE_POLL );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The FIR filter filters a block in the interrupt mode (the FMAC interrupt moves the samples).
 * \par Expected results
 * Same as in the POLL mode.
 */
void It_Fmac_Set_FilterProcess_FirIsr_ResultsMatchReference( void )
{
#if defined(FMAC)
    ItFmac_Run_Block( FMAC_XFER_MODE_ISR );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The FIR filter filters a block in the DMA mode (two Gpdma channels move the samples).
 * \par Expected results
 * Same as in the POLL mode.
 */
void It_Fmac_Set_FilterProcess_FirDma_ResultsMatchReference( void )
{
#if defined(FMAC)
    ItFmac_Run_Block( FMAC_XFER_MODE_DMA );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief Two blocks in a row keep the state of the FIR filter: the second block continues the first.
 * \par Expected results
 * The results of the second block are those of the reference calculation of the 16 samples (the first 8 samples are
 * the history of the second block).
 */
void It_Fmac_Set_FilterProcess_FirTwoBlocks_KeepsState( void )
{
#if defined(FMAC)
    const fmac_FilterConfig_t filterConfig = ItFmac_Get_FirConfig();
    fmac_Sample_t             twoBlocks[ IT_FMAC_BLOCK_CNT * IT_FMAC_SAMPLE_CNT ];
    fmac_Sample_t             expectedSecond[ IT_FMAC_SAMPLE_CNT ];

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Start( FMAC_XFER_MODE_POLL, &filterConfig ) );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Process( itFmac_Input ) );
    ItFmac_Wait_Idle();

    for( uint32_t sampleIdx = 0u; ( IT_FMAC_SAMPLE_CNT > sampleIdx ); sampleIdx++ )
    {
        twoBlocks[ sampleIdx ]                      = itFmac_Input[ sampleIdx ];
        twoBlocks[ IT_FMAC_SAMPLE_CNT + sampleIdx ] = itFmac_Input[ sampleIdx ];
    }

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Process( itFmac_Input ) );
    ItFmac_Wait_Idle();

    ItFmac_Ref_Fir( itFmac_FirCoef, IT_FMAC_TAPS, twoBlocks, ( IT_FMAC_BLOCK_CNT * IT_FMAC_SAMPLE_CNT ) );

    for( uint32_t sampleIdx = 0u; ( IT_FMAC_SAMPLE_CNT > sampleIdx ); sampleIdx++ )
    {
        expectedSecond[ sampleIdx ] = itFmac_Expected[ IT_FMAC_SAMPLE_CNT + sampleIdx ];
    }

    TEST_ASSERT_EQUAL_INT16_ARRAY( expectedSecond, itFmac_Output, IT_FMAC_SAMPLE_CNT );
    TEST_ASSERT_EQUAL_UINT32( IT_FMAC_BLOCK_CNT, itFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The IIR filter filters a block in the POLL mode.
 * \par Expected results
 * The 8 results are those of the reference calculation (a first order low pass: 1024, 1536, 1792 ...), no error is
 * reported.
 */
void It_Fmac_Set_FilterProcess_IirPoll_ResultsMatchReference( void )
{
#if defined(FMAC)
    const fmac_FilterConfig_t filterConfig = ItFmac_Get_IirConfig();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Start( FMAC_XFER_MODE_POLL, &filterConfig ) );
    ItFmac_Ref_Iir();

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Process( itFmac_IirInput ) );
    ItFmac_Wait_Idle();

    ItFmac_Assert_Result( IT_FMAC_SAMPLE_CNT );
    TEST_ASSERT_EQUAL_UINT32( 1u, itFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The dot product of a vector of 4 inputs and the coefficient vector in the POLL mode.
 * \par Expected results
 * One result equal to the reference calculation.
 */
void It_Fmac_Set_FilterProcess_DotProductPoll_ResultMatchesReference( void )
{
#if defined(FMAC)
    ItFmac_Run_Vector( FMAC_XFER_MODE_POLL, FMAC_FUNC_DOT_PRODUCT );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The convolution of an input vector with the coefficient vector in the POLL mode.
 * \par Expected results
 * The 8 results are those of the reference calculation (zero state).
 */
void It_Fmac_Set_FilterProcess_ConvolutionPoll_ResultsMatchReference( void )
{
#if defined(FMAC)
    ItFmac_Run_Vector( FMAC_XFER_MODE_POLL, FMAC_FUNC_CONVOLUTION );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The correlation of an input vector with the coefficient vector in the POLL mode.
 * \par Expected results
 * The 8 results are those of the reference calculation (zero state).
 */
void It_Fmac_Set_FilterProcess_CorrelationPoll_ResultsMatchReference( void )
{
#if defined(FMAC)
    ItFmac_Run_Vector( FMAC_XFER_MODE_POLL, FMAC_FUNC_CORRELATION );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The convolution in the DMA mode arms both Gpdma channels and gives the same results as in the POLL mode.
 * \par Expected results
 * The 8 results are those of the reference calculation, the complete callback is called once.
 */
void It_Fmac_Set_FilterProcess_ConvolutionDma_ResultsMatchReference( void )
{
#if defined(FMAC)
    ItFmac_Run_Vector( FMAC_XFER_MODE_DMA, FMAC_FUNC_CONVOLUTION );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The dot product in the DMA mode reads its result by the Gpdma.
 * \par Expected results
 * One result equal to the reference calculation, the complete callback is called once.
 */
void It_Fmac_Set_FilterProcess_DotProductDma_ResultMatchesReference( void )
{
#if defined(FMAC)
    ItFmac_Run_Vector( FMAC_XFER_MODE_DMA, FMAC_FUNC_DOT_PRODUCT );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief A one shot stream ends after one pass through the buffers in the DMA mode.
 * \par Expected results
 * The 8 results are those of the reference calculation, the half and the complete callback are called once, the
 * transfer state is idle and the filter keeps running.
 */
void It_Fmac_Set_StreamStart_OneShotDma_ResultsMatchReference( void )
{
#if defined(FMAC)
    const fmac_FilterConfig_t   filterConfig  = ItFmac_Get_FirConfig();
    const fmac_StreamRequest_t  streamRequest =
    {
        .Input      = itFmac_Input,
        .InputSize  = IT_FMAC_SAMPLE_CNT,
        .Output     = itFmac_Output,
        .OutputSize = IT_FMAC_SAMPLE_CNT,
        .BufferMode = FMAC_BUFFER_ONE_SHOT,
    };
    fmac_FilterState_t          filterState   = FMAC_FILTER_STOPPED;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Start( FMAC_XFER_MODE_DMA, &filterConfig ) );
    ItFmac_Ref_Fir( itFmac_FirCoef, IT_FMAC_TAPS, itFmac_Input, IT_FMAC_SAMPLE_CNT );

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_StreamStart( &streamRequest ) );
    ItFmac_Wait_Idle();

    ItFmac_Assert_Result( IT_FMAC_SAMPLE_CNT );
    TEST_ASSERT_EQUAL_UINT32( 1u, itFmac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_RUNNING, filterState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_ERROR, Fmac_Set_StreamStop() );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief A circular stream in the DMA mode runs several laps until it is stopped.
 * \par Expected results
 * At least 3 complete callbacks and 3 half callbacks and no error (the output buffer is rewritten by every lap, so the
 * test checks the counters); after \ref Fmac_Set_StreamStop the transfer state is idle, the filter is stopped and no
 * further callback is called. The buffers have 512 samples: with 8 samples the Gpdma interrupt of every lap kept the
 * CPU busy so that the test never reached the stop.
 */
void It_Fmac_Set_StreamStart_CircularDma_RunsLapsUntilStopped( void )
{
#if defined(FMAC)
    const fmac_FilterConfig_t   filterConfig  = ItFmac_Get_FirConfig();
    const fmac_StreamRequest_t  streamRequest =
    {
        .Input      = itFmac_StreamInput,
        .InputSize  = IT_FMAC_STREAM_SAMPLE_CNT,
        .Output     = itFmac_StreamOutput,
        .OutputSize = IT_FMAC_STREAM_SAMPLE_CNT,
        .BufferMode = FMAC_BUFFER_CIRCULAR,
    };
    fmac_XferState_t            xferState     = FMAC_XFER_STATE_BUSY;
    fmac_FilterState_t          filterState   = FMAC_FILTER_RUNNING;
    uint32_t                    lapCnt        = 0u;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Start( FMAC_XFER_MODE_DMA, &filterConfig ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_StreamStart( &streamRequest ) );

    ItFmac_Wait_Laps( IT_FMAC_STREAM_LAPS );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_StreamStop() );

    lapCnt = itFmac_CompleteCnt;
    TEST_ASSERT_TRUE( IT_FMAC_STREAM_LAPS <= lapCnt );
    TEST_ASSERT_TRUE( IT_FMAC_STREAM_LAPS <= itFmac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_XferState( &xferState ) );
    TEST_ASSERT_EQUAL( FMAC_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
    TEST_ASSERT_EQUAL_UINT32( lapCnt, itFmac_CompleteCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief A circular stream in the POLL mode runs one lap per Fmac_Task and stops on request.
 * \par Expected results
 * At least 3 complete callbacks after the laps served by \ref Fmac_Task, no error, the stream stops
 * (\ref Fmac_Set_StreamStop returns \ref FMAC_REQUEST_OK, the filter is stopped).
 */
void It_Fmac_Set_StreamStart_CircularPoll_RunsLapsUntilStopped( void )
{
#if defined(FMAC)
    const fmac_FilterConfig_t   filterConfig  = ItFmac_Get_FirConfig();
    const fmac_StreamRequest_t  streamRequest =
    {
        .Input      = itFmac_Input,
        .InputSize  = IT_FMAC_SAMPLE_CNT,
        .Output     = itFmac_Output,
        .OutputSize = IT_FMAC_SAMPLE_CNT,
        .BufferMode = FMAC_BUFFER_CIRCULAR,
    };
    fmac_FilterState_t          filterState   = FMAC_FILTER_RUNNING;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, ItFmac_Start( FMAC_XFER_MODE_POLL, &filterConfig ) );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_StreamStart( &streamRequest ) );

    ItFmac_Wait_Laps( IT_FMAC_STREAM_LAPS );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Set_StreamStop() );

    TEST_ASSERT_TRUE( IT_FMAC_STREAM_LAPS <= itFmac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itFmac_ErrorCnt );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FilterState( &filterState ) );
    TEST_ASSERT_EQUAL( FMAC_FILTER_STOPPED, filterState );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}


/**
 * \brief The conversion between floating point values and q1.15 samples.
 * \par Expected results
 * 0.5 is the sample 16384 and back, a value above the range is saturated to 32767.
 */
void It_Fmac_Get_FromFloat_ConvertsAndSaturates( void )
{
#if defined(FMAC)
    fmac_Sample_t sample = 0;
    float         value  = 0.0f;

    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( IT_FMAC_FLOAT_HALF, &sample ) );
    TEST_ASSERT_EQUAL_INT16( IT_FMAC_SAMPLE_HALF, sample );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_ToFloat( sample, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( IT_FMAC_FLOAT_HALF, value );
    TEST_ASSERT_EQUAL( FMAC_REQUEST_OK, Fmac_Get_FromFloat( IT_FMAC_FLOAT_TOO_LARGE, &sample ) );
    TEST_ASSERT_EQUAL_INT16( IT_FMAC_SAMPLE_MAX, sample );
#else
    TEST_IGNORE_MESSAGE( IT_FMAC_NO_FMAC_MSG );
#endif /* FMAC */
}
