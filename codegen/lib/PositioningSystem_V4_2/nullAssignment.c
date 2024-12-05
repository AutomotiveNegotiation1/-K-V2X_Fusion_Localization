/*
 * nullAssignment.c
 *
 * Code generation for function 'nullAssignment'
 *
 */

/* Include files */
#include "nullAssignment.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void nullAssignment(creal_T x_data[], int x_size[2], const int idx_data[],
                    const int idx_size[2])
{
  int b_size_idx_1;
  int k;
  int k0;
  int nxout;
  boolean_T b_data[12];
  b_size_idx_1 = x_size[1];
  nxout = x_size[1];
  if (nxout - 1 >= 0) {
    memset(&b_data[0], 0, (unsigned int)nxout * sizeof(boolean_T));
  }
  nxout = idx_size[1];
  for (k = 0; k < nxout; k++) {
    b_data[idx_data[k] - 1] = true;
  }
  nxout = 0;
  k0 = -1;
  for (k = 0; k < b_size_idx_1; k++) {
    boolean_T b;
    b = b_data[k];
    nxout += b;
    if ((k + 1 > b_size_idx_1) || (!b)) {
      k0++;
      x_data[k0] = x_data[k];
    }
  }
  nxout = x_size[1] - nxout;
  if (nxout < 1) {
    x_size[1] = 0;
  } else {
    x_size[1] = nxout;
  }
}

/* End of code generation (nullAssignment.c) */
