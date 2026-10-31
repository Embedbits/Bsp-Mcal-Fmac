/**
 * \author Mr.Nobody
 * \file Fmac_Port.h
 * \ingroup Fmac
 * \brief Fmac module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not used outside of the module.
 *
 */

#ifndef FMAC_FMAC_PORT_H
#define FMAC_FMAC_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Fmac_Types.h"                     /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

fmac_ModuleVersion_t        Fmac_Get_ModuleVersion          ( void );

fmac_RequestState_t         Fmac_Init                       ( const fmac_Config_t * const fmacConfig );
fmac_RequestState_t         Fmac_Deinit                     ( void );
void                        Fmac_Task                       ( void );

fmac_RequestState_t         Fmac_Get_DefaultConfig          ( fmac_Config_t * const fmacConfig );

/* ------------------------------- Filter ----------------------------------- */

fmac_RequestState_t         Fmac_Get_MemoryUsage            ( const fmac_FilterConfig_t * const filterConfig,
                                                              fmac_SampleCnt_t * const wordCnt );
fmac_RequestState_t         Fmac_Set_FilterConfig           ( const fmac_FilterConfig_t * const filterConfig );
fmac_RequestState_t         Fmac_Set_FilterStart            ( void );
fmac_RequestState_t         Fmac_Set_FilterStop             ( void );
fmac_RequestState_t         Fmac_Set_FilterPreload          ( const fmac_Sample_t * const inputHistory,
                                                              fmac_SampleCnt_t inputCnt,
                                                              const fmac_Sample_t * const outputHistory,
                                                              fmac_SampleCnt_t outputCnt );
fmac_RequestState_t         Fmac_Get_FilterState            ( fmac_FilterState_t * const filterState );

/* ------------------------------ Processing -------------------------------- */

fmac_RequestState_t         Fmac_Set_FilterProcess          ( const fmac_Sample_t * const input,
                                                              fmac_SampleCnt_t inputCnt,
                                                              fmac_Sample_t * const output,
                                                              fmac_SampleCnt_t outputCnt );
fmac_RequestState_t         Fmac_Set_StreamStart            ( const fmac_StreamRequest_t * const streamRequest );
fmac_RequestState_t         Fmac_Set_StreamStop             ( void );
fmac_RequestState_t         Fmac_Get_XferState              ( fmac_XferState_t * const xferState );
fmac_RequestState_t         Fmac_Get_XferError              ( fmac_XferError_t * const xferError );

/* ------------------------------ Conversion -------------------------------- */

fmac_RequestState_t         Fmac_Get_FromFloat              ( float value, fmac_Sample_t * const sample );
fmac_RequestState_t         Fmac_Get_ToFloat                ( fmac_Sample_t sample, float * const value );

/* ------------------------------- Interrupts ------------------------------- */

fmac_RequestState_t         Fmac_Set_IrqPriority            ( fmac_IrqPrio_t irqPrio );
fmac_RequestState_t         Fmac_Get_IrqPriority            ( fmac_IrqPrio_t * const irqPrio );

#ifdef __cplusplus
}
#endif

#endif /* FMAC_FMAC_PORT_H */
