/*
 * UWBMultiTagPos_V3_1.h
 *
 * Code generation for function 'UWBMultiTagPos_V3_1'
 *
 */

#ifndef UWBMULTITAGPOS_V3_1_H
#define UWBMULTITAGPOS_V3_1_H

/* Include files */
#include "PositioningSystem_V4_2_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double UWBMultiTagPos_V3_1(const cell_wrap_4 b_PosUWB2[4],
                           const double xt_b_data[], const int xt_b_size[2],
                           const double yt_b_data[], const int yt_b_size[2],
                           const double xain_data[], const int xain_size[2],
                           const double yain_data[], const int yain_size[2],
                           const emxArray_real_T *b_DistMap, double Ln,
                           creal_T *PosHH);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (UWBMultiTagPos_V3_1.h) */
