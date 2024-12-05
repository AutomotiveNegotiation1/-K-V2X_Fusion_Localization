/*
 * _coder_PositioningSystem_V4_2_mex.c
 *
 * Code generation for function 'PositioningSystem_V4_2'
 *
 */

/* Include files */
#include "_coder_PositioningSystem_V4_2_mex.h"
#include "_coder_PositioningSystem_V4_2_api.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&PositioningSystem_V4_2_atexit);
  /* Module initialization. */
  PositioningSystem_V4_2_initialize();
  /* Dispatch the entry-point. */
  unsafe_PositioningSystem_V4_2_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  PositioningSystem_V4_2_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-949", true);
  return emlrtRootTLSGlobal;
}

void unsafe_PositioningSystem_V4_2_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                               int32_T nrhs,
                                               const mxArray *prhs[1])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        22, "PositioningSystem_V4_2");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 22,
                        "PositioningSystem_V4_2");
  }
  /* Call the function. */
  PositioningSystem_V4_2_api(prhs[0], &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/* End of code generation (_coder_PositioningSystem_V4_2_mex.c) */