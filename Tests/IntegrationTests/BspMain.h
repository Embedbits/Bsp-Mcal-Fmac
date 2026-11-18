/**
 * \author Mr.Nobody
 * \file BspMain.h
 * \ingroup Fmac
 * \brief Entry point of Fmac integration test firmware (called by StartUp).
 *
 * StartUp calls BspMain() and gets this header through the interface library
 * BspMain_Lib (created by Tests/IntegrationTests/CMakeLists.txt). BspMain() is
 * implemented by ItTarget_Fmac.c.
 */

#ifndef FMAC_ITTEST_BSPMAIN_H
#define FMAC_ITTEST_BSPMAIN_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* ======================== EXPORTED FUNCTIONS ============================== */

void BspMain( void );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* FMAC_ITTEST_BSPMAIN_H */
