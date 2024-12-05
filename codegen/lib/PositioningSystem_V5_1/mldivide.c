/*
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 */

/* Include files */
#include "mldivide.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
void mldivide(const double A[4], const double B_data[], const int B_size[2],
              double Y_data[], int Y_size[2])
{
  int k;
  if (B_size[1] == 0) {
    Y_size[0] = 2;
    Y_size[1] = 0;
  } else {
    double a21;
    double a22;
    double a22_tmp;
    int nb;
    int r1;
    int r2;
    if (fabs(A[1]) > fabs(A[0])) {
      r1 = 1;
      r2 = 0;
    } else {
      r1 = 0;
      r2 = 1;
    }
    a21 = A[r2] / A[r1];
    a22_tmp = A[r1 + 2];
    a22 = A[r2 + 2] - a21 * a22_tmp;
    nb = B_size[1];
    Y_size[0] = 2;
    Y_size[1] = B_size[1];
    for (k = 0; k < nb; k++) {
      double d;
      double d1;
      d = B_data[r1 + 2 * k];
      d1 = (B_data[r2 + 2 * k] - d * a21) / a22;
      Y_data[2 * k + 1] = d1;
      Y_data[2 * k] = (d - d1 * a22_tmp) / A[r1];
    }
  }
}

/* End of code generation (mldivide.c) */
