/*
 * UWBpos_V2_3.h
 *
 * Code generation for function 'UWBpos_V2_3'
 *
 */

#ifndef UWBPOS_V2_3_H
#define UWBPOS_V2_3_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void UWBpos_V2_3(double Nanchor, const double RxIDin_data[],
                 const double RxDistin_data[], const double xain_data[],
                 const double yain_data[], creal_T *Pos1, creal_T Pos2_data[],
                 int Pos2_size[2], creal_T Pos3_data[], int Pos3_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (UWBpos_V2_3.h) */
