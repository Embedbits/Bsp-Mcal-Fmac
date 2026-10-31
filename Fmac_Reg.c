/**
 * \author Mr.Nobody
 * \file Fmac_Reg.c
 * \ingroup Fmac
 * \brief Fmac module Reg component functionality.
 *
 * Register access of the module: generic accessors (bounds checked register ID) and the register level
 * protocols of RM0481 / RM0517 (FMAC chapter): the unit reset, the buffer configuration, the start and the stop
 * of a function, the write of a sample and the read of a result. The registers are written only here, the masks
 * come from the CMSIS device header.
 *
 * The FMAC exists on STM32H562, H563, H573 and on the STM32H5E / H5F lines. On the other lines the register list is
 * not defined and the component is empty (the public functions of the module return an error there).
 *
 * After the last free place of the input buffer is filled, and after the last result of the output buffer is read,
 * the flags X1FULL and YEMPTY change with a delay of three clock cycles (RM0481 FMAC status register).
 * \ref Fmac_Reg_Write_Sample and \ref Fmac_Reg_Read_Sample therefore read FMAC_SR once and drop the value, the
 * following read of the flags then is not too early.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Fmac_Reg.h"                       /* Self include                   */
#include "Fmac_Types.h"                     /* Module types definitions       */
#include "Stm32.h"                          /* FMAC registers definition      */
/* ============================== TYPEDEFS ================================== */

#if defined(FMAC)

/** \brief Structure type used to store physical address of FMAC registers.
 * This array is used to reduce size of configuration array. With this style
 * the registers are referenced through enumeration.
 *
 */
typedef struct
{
    fmac_RegId_t        RegId;   /**< Peripheral register ID      */
    volatile uint32_t * RegAddr; /**< Peripheral register address */
}   fmac_RegList_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static uint32_t Fmac_Get_ParamValue( const fmac_RegFuncParam_t * const funcParam );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Register value without any bit set (masked register value of an inactive bit field) */
#define FMAC_REG_BITS_CLEAR              ( 0u )

/** \brief Count of read-back iterations of a register waiting for a hardware reaction */
#define FMAC_REG_TIMEOUT_CNT             ( 0x1000u )

/* The three buffer configuration registers share the position of the base and of the size fields */
_Static_assert( ( FMAC_X1BUFCFG_X1_BASE_Pos     == FMAC_X2BUFCFG_X2_BASE_Pos     ) &&
                ( FMAC_X1BUFCFG_X1_BASE_Pos     == FMAC_YBUFCFG_Y_BASE_Pos       ) &&
                ( FMAC_X1BUFCFG_X1_BUF_SIZE_Pos == FMAC_X2BUFCFG_X2_BUF_SIZE_Pos ) &&
                ( FMAC_X1BUFCFG_X1_BUF_SIZE_Pos == FMAC_YBUFCFG_Y_BUF_SIZE_Pos   ) &&
                ( FMAC_X1BUFCFG_FULL_WM_Pos     == FMAC_YBUFCFG_EMPTY_WM_Pos     ),
                "Fmac_Reg: buffer configuration registers have different field positions." );

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief FMAC registers configuration structure
 *
 * This structure is used to store addresses of FMAC registers.
 * It is used to reduce code size while referring to FMAC registers.
 */
static const fmac_RegList_t             fmac_RegList[] =
{
    { .RegId = FMAC_REG_X1BUFCFG, .RegAddr = &(FMAC->X1BUFCFG) }, /**< FMAC X1 buffer configuration register */
    { .RegId = FMAC_REG_X2BUFCFG, .RegAddr = &(FMAC->X2BUFCFG) }, /**< FMAC X2 buffer configuration register */
    { .RegId = FMAC_REG_YBUFCFG , .RegAddr = &(FMAC->YBUFCFG ) }, /**< FMAC Y buffer configuration register  */
    { .RegId = FMAC_REG_PARAM   , .RegAddr = &(FMAC->PARAM   ) }, /**< FMAC parameter register               */
    { .RegId = FMAC_REG_CR      , .RegAddr = &(FMAC->CR      ) }, /**< FMAC control register                 */
    { .RegId = FMAC_REG_SR      , .RegAddr = &(FMAC->SR      ) }, /**< FMAC status register                  */
    { .RegId = FMAC_REG_WDATA   , .RegAddr = &(FMAC->WDATA   ) }, /**< FMAC write data register              */
    { .RegId = FMAC_REG_RDATA   , .RegAddr = &(FMAC->RDATA   ) }, /**< FMAC read data register               */
};

_Static_assert( (sizeof(fmac_RegList) / sizeof(fmac_RegList_t)) == FMAC_REG_CNT,
                "Fmac_Reg: fmac_RegList has incorrect size." );

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Sets the register bit with given mask.
 *
 * \param regId   [in]: FMAC register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void Fmac_Set_RegBit( fmac_RegId_t regId, uint32_t bitMask )
{
    if( FMAC_REG_CNT > regId )
    {
        *fmac_RegList[regId].RegAddr |= bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Resets the register bit with given mask.
 *
 * \param regId   [in]: FMAC register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void Fmac_Reset_RegBit( fmac_RegId_t regId, uint32_t bitMask )
{
    if( FMAC_REG_CNT > regId )
    {
        *fmac_RegList[regId].RegAddr &= ~bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register bit value with given mask.
 *
 * \param regId   [in]: FMAC register ID
 * \param bitMask [in]: Mask to apply to the register
 * \return Register bit value with applied mask, no bit set for invalid register ID
 */
uint32_t Fmac_Get_RegBit( fmac_RegId_t regId, uint32_t bitMask )
{
    uint32_t retValue = FMAC_REG_BITS_CLEAR;

    if( FMAC_REG_CNT > regId )
    {
        retValue = ( *fmac_RegList[regId].RegAddr & bitMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = FMAC_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with given mask and value.
 *
 * \param regId    [in]: FMAC register ID
 * \param regMask  [in]: Mask to apply to the register
 * \param regValue [in]: Value to set in the register
 */
void Fmac_Set_RegVal( fmac_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    if( FMAC_REG_CNT > regId )
    {
        *fmac_RegList[regId].RegAddr = ( ( *fmac_RegList[regId].RegAddr & ~regMask ) |
                                         ( regValue & regMask ) );
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register value with given mask.
 *
 * \param regId   [in]: FMAC register ID
 * \param regMask [in]: Mask to apply to the register
 * \return Register value with applied mask, no bit set for invalid register ID
 */
uint32_t Fmac_Get_RegVal( fmac_RegId_t regId, uint32_t regMask )
{
    uint32_t retValue = FMAC_REG_BITS_CLEAR;

    if( FMAC_REG_CNT > regId )
    {
        retValue = ( *fmac_RegList[regId].RegAddr & regMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = FMAC_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Waits until the register (bit field) reads back the required value.
 *
 * \param regId    [in]: FMAC register ID
 * \param regMask  [in]: Mask of the checked bits
 * \param regValue [in]: Required value of the masked bits
 *
 * \return Returns \ref FMAC_REQUEST_OK if the value was read back within the
 *         timeout. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Reg_Wait_RegVal( fmac_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

    for( uint32_t iterationCnt = 0u; FMAC_REG_TIMEOUT_CNT > iterationCnt; iterationCnt ++ )
    {
        const uint32_t actualValue = Fmac_Get_RegVal( regId, regMask );

        if( ( regValue & regMask ) == actualValue )
        {
            retState = FMAC_REQUEST_OK;
            break;
        }
        else
        {
            /* Register has not been updated yet */
            retState = FMAC_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Resets the FMAC unit (write and read pointers, control logic, FMAC_SR and FMAC_PARAM) and waits until the
 *        hardware clears the reset bit. The buffer configuration and the other control bits are not affected.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the reset bit was cleared within the timeout.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Reg_Reset_Unit( void )
{
    Fmac_Set_RegBit( FMAC_REG_CR, FMAC_CR_RESET );

    return ( Fmac_Reg_Wait_RegVal( FMAC_REG_CR, FMAC_CR_RESET, FMAC_REG_BITS_CLEAR ) );
}


/**
 * \brief Writes the base address, the size and the watermark of a buffer configuration register. The register can
 *        be written only while no function runs (FMAC_PARAM.START is cleared).
 *
 * \param regId  [in]: FMAC_X1BUFCFG, FMAC_X2BUFCFG or FMAC_YBUFCFG register ID
 * \param bufCfg [in]: Base address, size and watermark of the buffer. Must not be NULL.
 */
void Fmac_Reg_Set_BufCfg( fmac_RegId_t regId, const fmac_RegBufCfg_t * const bufCfg )
{
    if( ( FMAC_NULL_PTR != bufCfg ) &&
        ( ( FMAC_REG_X1BUFCFG == regId ) ||
          ( FMAC_REG_X2BUFCFG == regId ) ||
          ( FMAC_REG_YBUFCFG  == regId )    ) )
    {
        uint32_t regValue = ( ( ( bufCfg->Base << FMAC_X1BUFCFG_X1_BASE_Pos     ) & FMAC_X1BUFCFG_X1_BASE_Msk     ) |
                              ( ( bufCfg->Size << FMAC_X1BUFCFG_X1_BUF_SIZE_Pos ) & FMAC_X1BUFCFG_X1_BUF_SIZE_Msk )   );

        if( FMAC_REG_X2BUFCFG != regId )
        {
            regValue |= ( ( bufCfg->Watermark << FMAC_X1BUFCFG_FULL_WM_Pos ) & FMAC_X1BUFCFG_FULL_WM_Msk );
        }
        else
        {
            /* The coefficient buffer has no watermark */
        }

        Fmac_Set_RegVal( regId, FMAC_REG_FULL_MASK, regValue );
    }
    else
    {
        /* Invalid register ID or parameters, no action required */
    }
}


/**
 * \brief Starts a function: writes FMAC_PARAM (function, P, Q, R) together with the START bit. A load function is
 *        completed by the hardware after the requested count of writes to FMAC_WDATA (see
 *        \ref Fmac_Reg_Wait_FunctionDone), a filter function runs until \ref Fmac_Reg_Stop_Function.
 *
 * \param funcParam [in]: Function and its parameters. Must not be NULL.
 */
void Fmac_Reg_Start_Function( const fmac_RegFuncParam_t * const funcParam )
{
    if( FMAC_NULL_PTR != funcParam )
    {
        const uint32_t regValue = ( Fmac_Get_ParamValue( funcParam ) | FMAC_PARAM_START );

        Fmac_Set_RegVal( FMAC_REG_PARAM, FMAC_REG_FULL_MASK, regValue );
    }
    else
    {
        /* Missing parameters, no action required */
    }
}


/**
 * \brief Waits until the function and its parameters (FUNC, R, Q, P) of FMAC_PARAM read back as written by
 *        \ref Fmac_Reg_Start_Function. The START bit is not part of the check: a vector function of a request in the
 *        interrupt or in the DMA mode is read out by the interrupt or by the Gpdma channel at once and its request
 *        stops the function (START reads back as cleared) before the check, which is not a failure of the start.
 *
 * \param funcParam [in]: Function and its parameters. Must not be NULL.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the fields read back as written within the timeout.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Reg_Wait_FunctionParam( const fmac_RegFuncParam_t * const funcParam )
{
    fmac_RequestState_t retState = FMAC_REQUEST_ERROR;

    if( FMAC_NULL_PTR != funcParam )
    {
        const uint32_t paramMask = ( FMAC_PARAM_FUNC_Msk | FMAC_PARAM_R_Msk | FMAC_PARAM_Q_Msk | FMAC_PARAM_P_Msk );

        retState = Fmac_Reg_Wait_RegVal( FMAC_REG_PARAM, paramMask, Fmac_Get_ParamValue( funcParam ) );
    }
    else
    {
        /* Missing parameters, nothing to compare with */
        retState = FMAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops the running function (clears the START bit) and waits until the bit reads back as cleared.
 *
 * \return Returns \ref FMAC_REQUEST_OK if the START bit is cleared. Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Reg_Stop_Function( void )
{
    Fmac_Reset_RegBit( FMAC_REG_PARAM, FMAC_PARAM_START );

    return ( Fmac_Reg_Wait_RegVal( FMAC_REG_PARAM, FMAC_PARAM_START, FMAC_REG_BITS_CLEAR ) );
}


/**
 * \brief Waits until the hardware clears the START bit (a load function got all its values).
 *
 * \return Returns \ref FMAC_REQUEST_OK if the START bit was cleared within the timeout.
 *         Otherwise returns \ref FMAC_REQUEST_ERROR.
 */
fmac_RequestState_t Fmac_Reg_Wait_FunctionDone( void )
{
    return ( Fmac_Reg_Wait_RegVal( FMAC_REG_PARAM, FMAC_PARAM_START, FMAC_REG_BITS_CLEAR ) );
}


/**
 * \brief Writes one sample to FMAC_WDATA and reads FMAC_SR once (the flags need three clock cycles to settle).
 *
 * \param sample [in]: Sample (q1.15)
 */
void Fmac_Reg_Write_Sample( fmac_Sample_t sample )
{
    const uint32_t rawValue = ( (uint32_t)(uint16_t)sample & FMAC_WDATA_WDATA_Msk );

    *fmac_RegList[ FMAC_REG_WDATA ].RegAddr = rawValue;

    /* The flags settle after three clock cycles, the value of this read is dropped */
    const uint32_t statusValue = *fmac_RegList[ FMAC_REG_SR ].RegAddr;

    (void)statusValue;
}


/**
 * \brief Reads one result from FMAC_RDATA and reads FMAC_SR once (the flags need three clock cycles to settle).
 *
 * \return Result (q1.15)
 */
fmac_Sample_t Fmac_Reg_Read_Sample( void )
{
    const uint32_t rawValue = ( *fmac_RegList[ FMAC_REG_RDATA ].RegAddr & FMAC_RDATA_RDATA_Msk );

    /* The flags settle after three clock cycles, the value of this read is dropped */
    const uint32_t statusValue = *fmac_RegList[ FMAC_REG_SR ].RegAddr;

    (void)statusValue;

    return ( (fmac_Sample_t)(uint16_t)rawValue );
}


/**
 * \brief Returns the address of FMAC_WDATA as a plain 32-bit value, for the Gpdma destination address.
 *
 * \return Address of FMAC_WDATA.
 */
uint32_t Fmac_Reg_Get_WdataAddr( void )
{
    return ( (uint32_t)(uintptr_t)fmac_RegList[ FMAC_REG_WDATA ].RegAddr );
}


/**
 * \brief Returns the address of FMAC_RDATA as a plain 32-bit value, for the Gpdma source address.
 *
 * \return Address of FMAC_RDATA.
 */
uint32_t Fmac_Reg_Get_RdataAddr( void )
{
    return ( (uint32_t)(uintptr_t)fmac_RegList[ FMAC_REG_RDATA ].RegAddr );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Builds the value of FMAC_PARAM of a function without the START bit.
 *
 * \param funcParam [in]: Function and its parameters. Must not be NULL.
 *
 * \return FUNC, R, Q and P fields of FMAC_PARAM.
 */
static uint32_t Fmac_Get_ParamValue( const fmac_RegFuncParam_t * const funcParam )
{
    return ( ( ( (uint32_t)funcParam->Func << FMAC_PARAM_FUNC_Pos ) & FMAC_PARAM_FUNC_Msk ) |
             ( ( funcParam->ParamR         << FMAC_PARAM_R_Pos    ) & FMAC_PARAM_R_Msk    ) |
             ( ( funcParam->ParamQ         << FMAC_PARAM_Q_Pos    ) & FMAC_PARAM_Q_Msk    ) |
             ( ( funcParam->ParamP         << FMAC_PARAM_P_Pos    ) & FMAC_PARAM_P_Msk    ) );
}

#endif /* FMAC */
