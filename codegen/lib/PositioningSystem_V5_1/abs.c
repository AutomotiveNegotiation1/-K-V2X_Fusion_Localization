/*
 * abs.c
 *
 * Code generation for function 'abs'
 *
 */

/* Include files */
#include "abs.h"
#include "PositioningSystem_V5_1_emxutil.h"
#include "PositioningSystem_V5_1_rtwutil.h"
#include "PositioningSystem_V5_1_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void b_abs(const emxArray_creal_T *x, emxArray_real_T *y)
{
  const creal_T *x_data;
  double *y_data;
  int k;
  int nx;
  x_data = x->data;
  nx = x->size[0] * x->size[1];
  k = y->size[0] * y->size[1];
  y->size[0] = x->size[0];
  y->size[1] = x->size[1];
  emxEnsureCapacity_real_T(y, k);
  y_data = y->data;
  for (k = 0; k < nx; k++) {
    y_data[k] = rt_hypotd_snf(x_data[k].re, x_data[k].im);
  }
}

/* End of code generation (abs.c) */
