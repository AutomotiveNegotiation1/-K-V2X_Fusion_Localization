/*
 * _coder_PositioningSystem_V5_1_api.h
 *
 * Code generation for function 'PositioningSystem_V5_1'
 *
 */

#ifndef _CODER_POSITIONINGSYSTEM_V5_1_API_H
#define _CODER_POSITIONINGSYSTEM_V5_1_API_H

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
void PositioningSystem_V5_1(real_T PositionVector_data[],
                            int32_T PositionVector_size[2],
                            real_T PositionOut[10]);

void PositioningSystem_V5_1_api(const mxArray *prhs, const mxArray **plhs);

void PositioningSystem_V5_1_atexit(void);

void PositioningSystem_V5_1_initialize(void);

void PositioningSystem_V5_1_terminate(void);

void PositioningSystem_V5_1_xil_shutdown(void);

void PositioningSystem_V5_1_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (_coder_PositioningSystem_V5_1_api.h) */
