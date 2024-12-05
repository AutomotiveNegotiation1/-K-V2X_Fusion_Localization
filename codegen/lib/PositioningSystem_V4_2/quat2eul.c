/*
 * quat2eul.c
 *
 * Code generation for function 'quat2eul'
 *
 */

/* Include files */
#include "quat2eul.h"
#include "PositioningSystem_V4_2_rtwutil.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static void binary_expand_op(double in1[3], const signed char in2_data[],
                             const int in2_size[2], const double in3_data[],
                             const double in4_data[]);

/* Function Definitions */
static void binary_expand_op(double in1[3], const signed char in2_data[],
                             const int in2_size[2], const double in3_data[],
                             const double in4_data[])
{
  int i;
  int loop_ub;
  loop_ub = in2_size[1];
  for (i = 0; i < loop_ub; i++) {
    in1[in2_data[0]] = -in3_data[0] * 2.0 * in4_data[0];
  }
}

void quat2eul(double q[4], double eul[3])
{
  double aSinInput;
  double b_eul_tmp;
  double c_eul_tmp;
  double d_eul_tmp;
  double eul_tmp;
  double unnamed_idx_0;
  int tmp_size[2];
  int b_trueCount;
  int k;
  int trueCount;
  signed char tmp_data;
  boolean_T b;
  aSinInput =
      1.0 / sqrt(((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3]);
  q[0] *= aSinInput;
  q[1] *= aSinInput;
  q[2] *= aSinInput;
  q[3] *= aSinInput;
  aSinInput = -2.0 * (q[1] * q[3] - q[0] * q[2]);
  unnamed_idx_0 = aSinInput;
  if (aSinInput >= 0.99999999999999778) {
    unnamed_idx_0 = 1.0;
  }
  if (aSinInput <= -0.99999999999999778) {
    unnamed_idx_0 = -1.0;
  }
  eul_tmp = q[0] * q[0];
  b_eul_tmp = q[1] * q[1];
  c_eul_tmp = q[2] * q[2];
  d_eul_tmp = q[3] * q[3];
  eul[0] = rt_atan2d_snf(2.0 * (q[1] * q[2] + q[0] * q[3]),
                         ((eul_tmp + b_eul_tmp) - c_eul_tmp) - d_eul_tmp);
  eul[1] = asin(unnamed_idx_0);
  eul[2] = rt_atan2d_snf(2.0 * (q[2] * q[3] + q[0] * q[1]),
                         ((eul_tmp - b_eul_tmp) - c_eul_tmp) + d_eul_tmp);
  trueCount = 0;
  b = ((aSinInput >= 0.99999999999999778) ||
       (aSinInput <= -0.99999999999999778));
  if (b) {
    trueCount = 1;
  }
  b_trueCount = 0;
  if (b) {
    b_trueCount = 1;
  }
  for (k = 0; k < b_trueCount; k++) {
    if (rtIsNaN(unnamed_idx_0)) {
      unnamed_idx_0 = rtNaN;
    } else if (unnamed_idx_0 < 0.0) {
      unnamed_idx_0 = -1.0;
    } else {
      unnamed_idx_0 = (unnamed_idx_0 > 0.0);
    }
  }
  b_trueCount = 0;
  if (b) {
    b_trueCount = 1;
  }
  for (k = 0; k < b_trueCount; k++) {
    aSinInput = rt_atan2d_snf(q[1], q[0]);
  }
  k = 0;
  if (b) {
    k = 1;
  }
  tmp_size[0] = 1;
  tmp_size[1] = k;
  if (b) {
    tmp_data = 0;
  }
  if (trueCount == b_trueCount) {
    if (k - 1 >= 0) {
      eul[0] = -unnamed_idx_0 * 2.0 * aSinInput;
    }
  } else {
    binary_expand_op(eul, (signed char *)&tmp_data, tmp_size,
                     (double *)&unnamed_idx_0, (double *)&aSinInput);
  }
  trueCount = 0;
  if (b) {
    trueCount = 1;
  }
  if (trueCount - 1 >= 0) {
    eul[2] = 0.0;
  }
}

/* End of code generation (quat2eul.c) */
