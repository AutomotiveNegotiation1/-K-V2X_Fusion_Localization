/*
 * UWBPosition_V4_1.h
 *
 * Code generation for function 'UWBPosition_V4_1'
 *
 */

#ifndef UWBPOSITION_V4_1_H
#define UWBPOSITION_V4_1_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double UWBPosition_V4_1(double s_time, double Ln, double Nanchor, double TagID,
                        const double RxIDUWB_data[], const int RxIDUWB_size[2],
                        const double RxDistOrig_data[],
                        const int RxDistOrig_size[2], const double xain_data[],
                        const int xain_size[2], const double yain_data[],
                        const int yain_size[2], const double zain_data[],
                        const double xt_b_data[], const int xt_b_size[2],
                        const double yt_b_data[], const int yt_b_size[2],
                        double zt_b, creal_T *PosUWBO, double *UWBFull);

void UWBPosition_V4_1_free(void);

void UWBPosition_V4_1_init(void);

void binary_expand_op_1(creal_T in1_data[], int in1_size[2],
                        const double in2_data[], const int in2_size[2],
                        const double in4_data[], const int in4_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (UWBPosition_V4_1.h) */
