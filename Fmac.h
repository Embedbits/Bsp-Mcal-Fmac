/**
 * \author Mr.Nobody
 * \file Fmac.h
 * \ingroup Fmac
 * \brief Fmac module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 *
 */

#ifndef FMAC_FMAC_H
#define FMAC_FMAC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Fmac_Types.h"                     /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/** \brief Layout of the FMAC local memory calculated from a filter configuration. Base addresses and sizes are in
 *         the 16-bit words of the local memory, the three areas do not overlap (the coefficients first, then the
 *         input buffer, then the output buffer). */
typedef struct
{
    fmac_SampleCnt_t X1Base; /**< Base address of the input buffer X1                                  */
    fmac_SampleCnt_t X1Size; /**< Size of the input buffer X1 (coefficients and the headroom)           */
    fmac_SampleCnt_t X2Base; /**< Base address of the coefficient buffer X2                            */
    fmac_SampleCnt_t X2Size; /**< Size of the coefficient buffer X2                                    */
    fmac_SampleCnt_t YBase;  /**< Base address of the output buffer Y                                  */
    fmac_SampleCnt_t YSize;  /**< Size of the output buffer Y (feedback coefficients and the headroom)  */
}   fmac_MemoryLayout_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

#ifdef __cplusplus
}
#endif

#endif /* FMAC_FMAC_H */
