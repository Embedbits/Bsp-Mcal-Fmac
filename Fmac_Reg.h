/**
 * \author Mr.Nobody
 * \file Fmac_Reg.h
 * \ingroup Fmac
 * \brief Fmac module Reg component header file.
 *
 * This component contains functionality for register operations. It is private
 * to the module (the register access is not shared with other modules).
 *
 */

#ifndef FMAC_REG_FMAC_REG_H
#define FMAC_REG_FMAC_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Fmac.h"                           /* Module internal types          */
#include "Fmac_Types.h"                     /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief List of FMAC registers accessed as a single 32-bit value */
typedef enum
{
    FMAC_REG_X1BUFCFG = 0u, /**< FMAC X1 buffer configuration register */
    FMAC_REG_X2BUFCFG     , /**< FMAC X2 buffer configuration register */
    FMAC_REG_YBUFCFG      , /**< FMAC Y buffer configuration register  */
    FMAC_REG_PARAM        , /**< FMAC parameter register               */
    FMAC_REG_CR           , /**< FMAC control register                 */
    FMAC_REG_SR           , /**< FMAC status register                  */
    FMAC_REG_WDATA        , /**< FMAC write data register              */
    FMAC_REG_RDATA        , /**< FMAC read data register               */
    FMAC_REG_CNT            /**< Count of available FMAC registers     */
}   fmac_RegId_t;


/** \brief Function codes of FMAC_PARAM.FUNC (RM0481 FMAC parameter register) */
typedef enum
{
    FMAC_REG_FUNC_LOAD_X1 = 1u, /**< Load the X1 buffer (P values)                          */
    FMAC_REG_FUNC_LOAD_X2 = 2u, /**< Load the X2 buffer (P + Q values)                      */
    FMAC_REG_FUNC_LOAD_Y  = 3u, /**< Load the Y buffer (P values)                           */
    FMAC_REG_FUNC_FIR     = 8u, /**< Convolution (FIR filter), runs until it is stopped      */
    FMAC_REG_FUNC_IIR     = 9u  /**< IIR filter (direct form 1), runs until it is stopped    */
}   fmac_RegFunc_t;


/** \brief Parameters of one FMAC function (FMAC_PARAM without the START bit), raw register field values */
typedef struct
{
    fmac_RegFunc_t Func;   /**< FMAC_PARAM.FUNC                                       */
    uint32_t       ParamP; /**< FMAC_PARAM.P (count of values or of feedforward taps) */
    uint32_t       ParamQ; /**< FMAC_PARAM.Q (count of feedback taps)                 */
    uint32_t       ParamR; /**< FMAC_PARAM.R (gain exponent)                          */
}   fmac_RegFuncParam_t;


/** \brief Parameters of one buffer configuration register (X1BUFCFG, X2BUFCFG, YBUFCFG), raw register field values */
typedef struct
{
    uint32_t Base;      /**< Base address in the 16-bit words of the local memory */
    uint32_t Size;      /**< Size in the 16-bit words of the local memory         */
    uint32_t Watermark; /**< FULL_WM / EMPTY_WM encoding (ignored for X2)         */
}   fmac_RegBufCfg_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** \brief Full register mask, used to write / read back a whole 32-bit register at once */
#define FMAC_REG_FULL_MASK               ( 0xFFFFFFFFu )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void                        Fmac_Set_RegBit             ( fmac_RegId_t regId, uint32_t bitMask );
void                        Fmac_Reset_RegBit           ( fmac_RegId_t regId, uint32_t bitMask );
uint32_t                    Fmac_Get_RegBit             ( fmac_RegId_t regId, uint32_t bitMask );

void                        Fmac_Set_RegVal             ( fmac_RegId_t regId, uint32_t regMask, uint32_t regValue );
uint32_t                    Fmac_Get_RegVal             ( fmac_RegId_t regId, uint32_t regMask );

fmac_RequestState_t         Fmac_Reg_Wait_RegVal        ( fmac_RegId_t regId, uint32_t regMask, uint32_t regValue );

/* -------------------------- Register level protocols ---------------------- */

fmac_RequestState_t         Fmac_Reg_Reset_Unit         ( void );
void                        Fmac_Reg_Set_BufCfg         ( fmac_RegId_t regId, const fmac_RegBufCfg_t * const bufCfg );
void                        Fmac_Reg_Start_Function     ( const fmac_RegFuncParam_t * const funcParam );
fmac_RequestState_t         Fmac_Reg_Wait_FunctionParam ( const fmac_RegFuncParam_t * const funcParam );
fmac_RequestState_t         Fmac_Reg_Stop_Function      ( void );
fmac_RequestState_t         Fmac_Reg_Wait_FunctionDone  ( void );
void                        Fmac_Reg_Write_Sample       ( fmac_Sample_t sample );
fmac_Sample_t               Fmac_Reg_Read_Sample        ( void );

/* ------------------------ Addresses for the DMA --------------------------- */

uint32_t                    Fmac_Reg_Get_WdataAddr      ( void );
uint32_t                    Fmac_Reg_Get_RdataAddr      ( void );

#ifdef __cplusplus
}
#endif

#endif /* FMAC_REG_FMAC_REG_H */
