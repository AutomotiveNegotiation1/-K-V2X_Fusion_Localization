/*
 * mean.c
 *
 * Code generation for function 'mean'
 *
 */

/* Include files */
#include "mean.h"
#include "PositioningSystem_V4_2_data.h"
#include "PositioningSystem_V4_2_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
creal_T mean(const emxArray_creal_T *x)
{
  const creal_T *x_data;
  creal_T y;
  double x_im;
  double x_re;
  int ib;
  int k;
  x_data = x->data;
  if (x->size[1] == 0) {
    x_re = 0.0;
    x_im = 0.0;
  } else {
    int firstBlockLength;
    int lastBlockLength;
    int nblocks;
    if (x->size[1] <= 1024) {
      firstBlockLength = x->size[1];
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = (int)((unsigned int)x->size[1] >> 10);
      lastBlockLength = x->size[1] - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    x_re = x_data[0].re;
    x_im = x_data[0].im;
    for (k = 2; k <= firstBlockLength; k++) {
      x_re += x_data[k - 1].re;
      x_im += x_data[k - 1].im;
    }
    for (ib = 2; ib <= nblocks; ib++) {
      double bsum_im;
      double bsum_re;
      int hi;
      firstBlockLength = (ib - 1) << 10;
      bsum_re = x_data[firstBlockLength].re;
      bsum_im = x_data[firstBlockLength].im;
      if (ib == nblocks) {
        hi = lastBlockLength;
      } else {
        hi = 1024;
      }
      for (k = 2; k <= hi; k++) {
        int bsum_re_tmp;
        bsum_re_tmp = (firstBlockLength + k) - 1;
        bsum_re += x_data[bsum_re_tmp].re;
        bsum_im += x_data[bsum_re_tmp].im;
      }
      x_re += bsum_re;
      x_im += bsum_im;
    }
  }
  if (x_im == 0.0) {
    y.re = x_re / (double)x->size[1];
    y.im = 0.0;
  } else if (x_re == 0.0) {
    y.re = 0.0;
    y.im = x_im / (double)x->size[1];
  } else {
    y.re = x_re / (double)x->size[1];
    y.im = x_im / (double)x->size[1];
  }
  return y;
}

/* End of code generation (mean.c) */
