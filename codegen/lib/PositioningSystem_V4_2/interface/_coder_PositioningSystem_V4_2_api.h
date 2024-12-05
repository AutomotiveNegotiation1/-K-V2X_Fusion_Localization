/*
 * _coder_PositioningSystem_V4_2_api.h
 *
 * Code generation for function 'PositioningSystem_V4_2'
 *
 */

#ifndef _CODER_POSITIONINGSYSTEM_V4_2_API_H
#define _CODER_POSITIONINGSYSTEM_V4_2_API_H

/* Include files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void PositioningSystem_V4_2(real_T PositionVector_data[],
                            int32_T PositionVector_size[2],
                            real_T PositionOut[10]);

void PositioningSystem_V4_2_api(const mxArray *prhs, const mxArray **plhs);

void PositioningSystem_V4_2_atexit(void);

void PositioningSystem_V4_2_initialize(void);

void PositioningSystem_V4_2_terminate(void);

void PositioningSystem_V4_2_xil_shutdown(void);

void PositioningSystem_V4_2_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (_coder_PositioningSystem_V4_2_api.h) */
