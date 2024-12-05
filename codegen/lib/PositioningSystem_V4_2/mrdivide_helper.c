/*
 * mrdivide_helper.c
 *
 * Code generation for function 'mrdivide_helper'
 *
 */

/* Include files */
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
void mrdiv(const double A[9], const double B[9], double Y[9])
{
  double b_A[9];
  double Y_tmp;
  double a21;
  double b_Y_tmp;
  double c_Y_tmp;
  double d_Y_tmp;
  double maxval;
  int e_Y_tmp;
  int f_Y_tmp;
  int r1;
  int r2;
  int r3;
  int rtemp;
  memcpy(&b_A[0], &B[0], 9U * sizeof(double));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(B[0]);
  a21 = fabs(B[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabs(B[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = B[r2] / B[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (fabs(b_A[r3 + 3]) > fabs(b_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  Y[3 * r1] = A[0] / b_A[r1];
  maxval = b_A[r1 + 3];
  Y[3 * r2] = A[3] - Y[3 * r1] * maxval;
  a21 = b_A[r1 + 6];
  Y[3 * r3] = A[6] - Y[3 * r1] * a21;
  Y_tmp = b_A[r2 + 3];
  Y[3 * r2] /= Y_tmp;
  b_Y_tmp = b_A[r2 + 6];
  Y[3 * r3] -= Y[3 * r2] * b_Y_tmp;
  c_Y_tmp = b_A[r3 + 6];
  Y[3 * r3] /= c_Y_tmp;
  d_Y_tmp = b_A[r3 + 3];
  Y[3 * r2] -= Y[3 * r3] * d_Y_tmp;
  Y[3 * r1] -= Y[3 * r3] * b_A[r3];
  Y[3 * r1] -= Y[3 * r2] * b_A[r2];
  rtemp = 3 * r1 + 1;
  Y[rtemp] = A[1] / b_A[r1];
  e_Y_tmp = 3 * r2 + 1;
  Y[e_Y_tmp] = A[4] - Y[rtemp] * maxval;
  f_Y_tmp = 3 * r3 + 1;
  Y[f_Y_tmp] = A[7] - Y[rtemp] * a21;
  Y[e_Y_tmp] /= Y_tmp;
  Y[f_Y_tmp] -= Y[e_Y_tmp] * b_Y_tmp;
  Y[f_Y_tmp] /= c_Y_tmp;
  Y[e_Y_tmp] -= Y[f_Y_tmp] * d_Y_tmp;
  Y[rtemp] -= Y[f_Y_tmp] * b_A[r3];
  Y[rtemp] -= Y[e_Y_tmp] * b_A[r2];
  rtemp = 3 * r1 + 2;
  Y[rtemp] = A[2] / b_A[r1];
  e_Y_tmp = 3 * r2 + 2;
  Y[e_Y_tmp] = A[5] - Y[rtemp] * maxval;
  f_Y_tmp = 3 * r3 + 2;
  Y[f_Y_tmp] = A[8] - Y[rtemp] * a21;
  Y[e_Y_tmp] /= Y_tmp;
  Y[f_Y_tmp] -= Y[e_Y_tmp] * b_Y_tmp;
  Y[f_Y_tmp] /= c_Y_tmp;
  Y[e_Y_tmp] -= Y[f_Y_tmp] * d_Y_tmp;
  Y[rtemp] -= Y[f_Y_tmp] * b_A[r3];
  Y[rtemp] -= Y[e_Y_tmp] * b_A[r2];
}

/* End of code generation (mrdivide_helper.c) */
